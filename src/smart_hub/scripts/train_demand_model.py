#!/usr/bin/env python3
"""Train and evaluate the S2 30-second demand forecaster (test T03).

Pipeline (all steps seeded and recorded):
  1. Generate N independent six-minute episodes (>= 30, default 50).
  2. Split WHOLE EPISODES 60/20/20 into train/val/test before any windowing.
  3. Run the same online estimator code the ROS node uses to produce
     feature rows (warm-up, gaps and stale input handled identically).
  4. For each forecast target ("demand" = unserved demand at t+30,
     "queue" = observed queue at t+30): fit the scaler on train only, choose
     the ridge alpha on val only.
  5. Report held-out test MAE/RMSE against the persistence baseline, per
     profile and per episode (including episodes where the model is worse),
     with a paired whole-episode bootstrap of the MAE difference.
  6. Measure inference latency through the online estimator (p50/p95).
  7. Compare reactive dispatch with predictive dispatch driven by each model
     on identical unseen seeds (passenger waits; censored reported apart).
  8. Export the shipped model JSON + split manifest + metrics + plots.

Usage:
  python3 scripts/train_demand_model.py            # defaults: target demand
  python3 scripts/train_demand_model.py --target queue
"""

import argparse
import csv
import datetime
import json
import os
import platform
import random
import statistics
import subprocess
import sys
import time

import numpy as np
import sklearn
from sklearn.linear_model import Ridge
from sklearn.preprocessing import StandardScaler

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from smart_hub.demand.estimator import DemandEstimator  # noqa: E402
from smart_hub.demand.features import FEATURE_NAMES  # noqa: E402
from smart_hub.demand.models import (  # noqa: E402
    PersistenceModel, build_linear_model, load_model)
from smart_hub.demand.simulation import (  # noqa: E402
    PROFILES, TARGETS, make_spec, run_episode, supervised_rows, wait_stats)

HORIZON = 30
ALPHAS = (0.01, 0.1, 1.0, 3.0, 10.0, 30.0, 100.0, 300.0)


# ---------------------------------------------------------------- utilities
def mae(y, p):
    return float(np.mean(np.abs(np.asarray(y) - np.asarray(p))))


def rmse(y, p):
    return float(np.sqrt(np.mean((np.asarray(y) - np.asarray(p)) ** 2)))


def pctl(xs, q):
    return float(np.percentile(np.asarray(xs, float), q)) if len(xs) else None


def git_commit():
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"], cwd=os.path.dirname(HERE),
            stderr=subprocess.DEVNULL).decode().strip()
    except Exception:  # noqa: BLE001
        return "not-a-git-repo"


def bootstrap_ci(values, n_boot=5000, seed=11):
    rng = np.random.default_rng(seed)
    v = np.asarray(values, float)
    boots = [v[rng.integers(0, len(v), len(v))].mean() for _ in range(n_boot)]
    return [pctl(boots, 2.5), pctl(boots, 97.5)]


def split_episodes(seeds, split_seed):
    """Stratified whole-episode 60/20/20 split, decided before windowing.

    Seeds are grouped by profile, shuffled within each profile, interleaved
    round-robin across profiles, then every block of five goes
    train/train/train/val/test. Every profile appears in every split and
    no episode contributes windows to more than one split.
    """
    by_profile = {p: [] for p in PROFILES}
    for s in seeds:
        by_profile[make_spec(s).profile].append(s)
    for p in PROFILES:
        random.Random("split-%d-%s" % (split_seed, p)).shuffle(by_profile[p])
    order = []
    while any(by_profile.values()):
        for p in PROFILES:
            if by_profile[p]:
                order.append(by_profile[p].pop(0))
    pattern = ("train", "train", "train", "val", "test")
    split = {"train": [], "val": [], "test": []}
    for i, s in enumerate(order):
        split[pattern[i % 5]].append(s)
    return split


def episode_rows(seeds):
    """Feature rows per episode and target, via the online estimator path."""
    per_ep = {}
    for s in seeds:
        res = run_episode(make_spec(s), PersistenceModel())
        per_ep[s] = {t: supervised_rows(res, HORIZON, t) for t in TARGETS}
    return per_ep


def stack(per_ep, seeds, target):
    X, y, base, ep = [], [], [], []
    for s in seeds:
        for feats, label, b, _t in per_ep[s][target]:
            X.append(feats)
            y.append(label)
            base.append(b)
            ep.append(s)
    return (np.asarray(X, float), np.asarray(y, float),
            np.asarray(base, float), np.asarray(ep))


# ------------------------------------------------------------ fit/evaluate
def fit_target(per_ep, split, target, model_path, extra_meta):
    Xtr, ytr, _, _ = stack(per_ep, split["train"], target)
    Xva, yva, bva, _ = stack(per_ep, split["val"], target)
    scaler = StandardScaler().fit(Xtr)            # train only
    val_table = []
    for a in ALPHAS:                              # alpha on validation only
        m = Ridge(alpha=a).fit(scaler.transform(Xtr), ytr)
        pv = np.maximum(0.0, m.predict(scaler.transform(Xva)))
        val_table.append({"alpha": a, "val_mae": mae(yva, pv),
                          "val_rmse": rmse(yva, pv)})
    best = min(val_table, key=lambda r: (r["val_mae"], -r["alpha"]))
    ridge = Ridge(alpha=best["alpha"]).fit(scaler.transform(Xtr), ytr)
    scale = [float(s) if s > 0 else 1.0 for s in scaler.scale_]
    prefix = "ridge-%s-h%d" % (target, HORIZON)
    meta = dict(extra_meta)
    meta.update({
        "created_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(timespec="seconds"),
        "target": target,
        "target_definition": TARGET_TEXT[target],
        "alpha": best["alpha"],
        "n_train_rows": int(len(ytr)),
        "n_val_rows": int(len(yva)),
    })
    model = build_linear_model(prefix, scaler.mean_.tolist(), scale,
                               ridge.coef_.tolist(), float(ridge.intercept_),
                               HORIZON, meta)
    with open(model_path, "w") as fh:
        json.dump(model.to_dict(), fh, indent=1)
    shipped = load_model(model_path)   # round-trip: evaluate what we ship

    Xte, _, _, _ = stack(per_ep, split["test"], target)
    sk = np.maximum(0.0, ridge.predict(scaler.transform(Xte)))
    py = np.asarray([shipped.predict(list(x)) for x in Xte])
    diff = float(np.max(np.abs(sk - py)))
    assert diff < 1e-6, diff
    return shipped, {
        "validation_alpha_table": val_table,
        "selected_alpha": best["alpha"],
        "val_persistence_mae": mae(yva, bva),
        "coefficients_standardized": dict(zip(FEATURE_NAMES,
                                              ridge.coef_.tolist())),
        "export_check_max_abs_diff_vs_sklearn": diff,
        "rows": {"train": int(len(ytr)), "val": int(len(yva))},
    }


def evaluate(model, per_ep, split, target):
    Xte, yte, bte, ete = stack(per_ep, split["test"], target)
    pte = np.asarray([model.predict(list(x)) for x in Xte])
    per_episode = []
    for s in sorted(split["test"]):
        m = ete == s
        per_episode.append({
            "seed": int(s), "profile": make_spec(s).profile, "n": int(m.sum()),
            "ridge_mae": mae(yte[m], pte[m]),
            "persistence_mae": mae(yte[m], bte[m]),
            "ridge_rmse": rmse(yte[m], pte[m]),
            "persistence_rmse": rmse(yte[m], bte[m]),
        })
    for e in per_episode:
        e["ridge_better"] = e["ridge_mae"] < e["persistence_mae"]
    # Paired whole-episode bootstrap of pooled MAE difference.
    rng = np.random.default_rng(7)
    sums = np.asarray([[np.abs(yte[ete == e["seed"]]
                               - pte[ete == e["seed"]]).sum(),
                        np.abs(yte[ete == e["seed"]]
                               - bte[ete == e["seed"]]).sum(), e["n"]]
                       for e in per_episode], float)
    boots = []
    for _ in range(5000):
        a = sums[rng.integers(0, len(sums), len(sums))]
        boots.append((a[:, 0].sum() - a[:, 1].sum()) / a[:, 2].sum())
    prof_of = np.asarray([make_spec(int(s)).profile for s in ete])
    per_profile = {}
    for p in PROFILES:
        m = prof_of == p
        if m.any():
            per_profile[p] = {"n": int(m.sum()),
                              "ridge_mae": mae(yte[m], pte[m]),
                              "persistence_mae": mae(yte[m], bte[m])}
    worst = []
    for i in np.argsort(-np.abs(yte - pte))[:5]:
        worst.append({
            "seed": int(ete[i]), "profile": str(prof_of[i]),
            "features": dict(zip(FEATURE_NAMES,
                                 [round(float(v), 2) for v in Xte[i]])),
            "label": float(yte[i]), "ridge": round(float(pte[i]), 2),
            "persistence": float(bte[i])})
    return {
        "target": target,
        "n_rows": int(len(yte)),
        "ridge_mae": mae(yte, pte), "ridge_rmse": rmse(yte, pte),
        "persistence_mae": mae(yte, bte), "persistence_rmse": rmse(yte, bte),
        "mae_difference_ridge_minus_persistence": mae(yte, pte)
        - mae(yte, bte),
        "mae_difference_95ci_episode_bootstrap": [pctl(boots, 2.5),
                                                  pctl(boots, 97.5)],
        "episodes_ridge_better": sum(e["ridge_better"] for e in per_episode),
        "episodes_ridge_worse": sum(not e["ridge_better"]
                                    for e in per_episode),
        "per_profile": per_profile,
        "per_episode": per_episode,
        "largest_ridge_errors": worst,
    }


def dispatch_study(seeds, test_seeds, arms):
    """Closed-loop: same passengers, different dispatch rule per arm."""
    rows = []
    for s in seeds:
        row = {"seed": s, "profile": make_spec(s).profile,
               "in_test_split": s in test_seeds}
        n_pass = None
        for arm, (mode, model) in arms.items():
            spec = make_spec(s)
            spec.bus_mode, spec.policy_mode = "policy", mode
            res = run_episode(spec, model)
            arrivals = len(res.passengers) + res.turned_away
            if n_pass is None:
                n_pass = arrivals
            assert n_pass == arrivals  # paired arrival stream in every arm
            for k, v in wait_stats(res).items():
                row["%s_%s" % (arm, k)] = v
            row["%s_fallback_dispatches" % arm] = sum(
                1 for d in res.dispatches if d["reason"] == "FALLBACK")
        row["arrivals"] = n_pass
        rows.append(row)

    summary = {"n_seeds": len(rows), "arms": {}}
    for arm in arms:
        waits = [r[arm + "_mean_wait_boarded"] for r in rows
                 if r[arm + "_mean_wait_boarded"] is not None]
        summary["arms"][arm] = {
            "mean_of_episode_mean_wait_sec": statistics.mean(waits),
            "total_boarded": sum(r[arm + "_boarded"] for r in rows),
            "total_censored": sum(r[arm + "_censored"] for r in rows),
            "total_left_queue": sum(r[arm + "_left_queue"] for r in rows),
            "total_dispatches": sum(r[arm + "_dispatches"] for r in rows),
            "fallback_dispatches": sum(r[arm + "_fallback_dispatches"]
                                       for r in rows),
        }
        if arm != "reactive":
            d = [r[arm + "_mean_wait_boarded"]
                 - r["reactive_mean_wait_boarded"] for r in rows
                 if r[arm + "_mean_wait_boarded"] is not None
                 and r["reactive_mean_wait_boarded"] is not None]
            summary["arms"][arm].update({
                "paired_diff_vs_reactive_sec": statistics.mean(d),
                "paired_diff_95ci_sec": bootstrap_ci(d),
                "episodes_lower_wait_than_reactive": sum(x < 0 for x in d),
                "episodes_higher_wait_than_reactive": sum(x > 0 for x in d),
                "episodes_compared": len(d)})
    return summary, rows


TARGET_TEXT = {
    "demand": "unserved demand at t+30 s: passengers who would be waiting "
              "at t+30 if no bus boarded after t (synthetic ground truth)",
    "queue": "true queue length at t+30 s, including the effect of any bus "
             "boarding in between (synthetic ground truth)",
}


# --------------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--base-seed", type=int, default=1000)
    ap.add_argument("--split-seed", type=int, default=42)
    ap.add_argument("--target", choices=TARGETS, default="demand",
                    help="forecast target of the shipped model")
    ap.add_argument("--policy-extra-seeds", type=int, default=40,
                    help="additional never-used seeds for the dispatch study")
    ap.add_argument("--out-dir", default=os.path.join(HERE, "..", "results"))
    ap.add_argument("--model-out",
                    default=os.path.join(HERE, "..", "models",
                                         "demand_ridge_h30.json"))
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args(argv)
    if args.episodes < 30:
        ap.error("the assignment requires at least 30 episodes")
    os.makedirs(args.out_dir, exist_ok=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.model_out)),
                exist_ok=True)
    t_start = time.time()

    # 1-2. episodes and whole-episode split ---------------------------------
    seeds = [args.base_seed + i for i in range(args.episodes)]
    split = split_episodes(seeds, args.split_seed)
    for a, b in (("train", "val"), ("train", "test"), ("val", "test")):
        assert not set(split[a]) & set(split[b]), (a, b)
    manifest = {
        "horizon_sec": HORIZON,
        "episode_duration_sec": 360,
        "base_seed": args.base_seed,
        "split_seed": args.split_seed,
        "split_rule": "stratified by profile, whole episodes, 60/20/20, "
                      "decided before windowing",
        "splits": {k: sorted(v) for k, v in split.items()},
        "episodes": {s: vars(make_spec(s)) for s in seeds},
    }
    with open(os.path.join(args.out_dir, "split_manifest.json"), "w") as fh:
        json.dump(manifest, fh, indent=1, default=str)

    # 3. rows via the online estimator ---------------------------------------
    per_ep = episode_rows(seeds)
    meta = {
        "train_seeds": sorted(split["train"]),
        "val_seeds": sorted(split["val"]),
        "test_seeds": sorted(split["test"]),
        "split_seed": args.split_seed,
        "sklearn": sklearn.__version__, "numpy": np.__version__,
        "python": platform.python_version(), "code_commit": git_commit(),
        "data_source": "synthetic seeded episodes "
                       "(smart_hub/demand/simulation.py)",
        "features": "count-delta proxies; see smart_hub/demand/features.py",
    }

    # 4-5. fit both targets, evaluate each on its own held-out labels -------
    models, fits, evals = {}, {}, {}
    for target in TARGETS:
        path = (args.model_out if target == args.target else
                os.path.join(args.out_dir, "alt_ridge_%s_h30.json" % target))
        models[target], fits[target] = fit_target(per_ep, split, target,
                                                  path, meta)
        evals[target] = evaluate(models[target], per_ep, split, target)
    shipped = models[args.target]
    # Cross-check: shipped model scored against the other target as well.
    other = [t for t in TARGETS if t != args.target][0]
    cross = evaluate(shipped, per_ep, split, other)

    # 6. inference latency through the online estimator path ----------------
    lat = []
    for s in split["test"]:
        res = run_episode(make_spec(s), shipped)
        lat += [r["inference_ms"] for r in res.ticks if r["inference_ms"]]
    est = DemandEstimator(shipped)
    tick_lat = []
    for k in range(2000):
        est.on_queue(k % 9, True, 0.9, float(k), float(k), float(k))
        t0 = time.perf_counter()
        est.tick(float(k) + 0.05, float(k) + 0.05)
        tick_lat.append((time.perf_counter() - t0) * 1000.0)

    # 7. dispatch study: identical seeds, three arms ------------------------
    policy_seeds = sorted(split["test"]) + [
        90000 + i for i in range(args.policy_extra_seeds)]
    arms = {"reactive": ("reactive", PersistenceModel())}
    for target in TARGETS:
        arms["predictive_" + target] = ("predictive", models[target])
    policy_summary, policy_rows = dispatch_study(policy_seeds,
                                                 set(split["test"]), arms)
    policy_summary["seeds_note"] = (
        "%d test-split seeds + %d never-used seeds; identical passenger "
        "arrivals in every arm" % (len(split["test"]),
                                   args.policy_extra_seeds))

    # 8. outputs ---------------------------------------------------------------
    ev = evals[args.target]
    pred_arm = policy_summary["arms"]["predictive_" + args.target]
    metrics = {
        "shipped_model_id": shipped.model_id,
        "shipped_target": args.target,
        "target_definition": TARGET_TEXT[args.target],
        "baseline_id": PersistenceModel.model_id,
        "horizon_sec": HORIZON,
        "episodes": {k: len(v) for k, v in split.items()},
        "fit": fits,
        "test": evals,
        "shipped_model_scored_on_other_target": cross,
        "latency_ms": {
            "features_plus_predict_p50": pctl(lat, 50),
            "features_plus_predict_p95": pctl(lat, 95),
            "features_plus_predict_n": len(lat),
            "full_tick_p50": pctl(tick_lat, 50),
            "full_tick_p95": pctl(tick_lat, 95),
            "full_tick_n": len(tick_lat),
            "machine": "%s %s, Python %s (pure-Python inference path)" % (
                platform.system(), platform.machine(),
                platform.python_version()),
        },
        "dispatch_policy_comparison": policy_summary,
        "pass_criteria": {
            "T03_ridge_mae_le_persistence":
                ev["ridge_mae"] <= ev["persistence_mae"],
            "T03_inference_p95_lt_100ms": pctl(lat, 95) < 100.0,
            "at_least_30_episodes": len(seeds) >= 30,
            "documented_worse_case_exists":
                ev["episodes_ridge_worse"] > 0
                or bool(ev["largest_ridge_errors"]),
            "predictive_wait_le_reactive":
                pred_arm["paired_diff_vs_reactive_sec"] <= 0,
        },
        "run_seconds": round(time.time() - t_start, 1),
    }
    with open(os.path.join(args.out_dir, "metrics.json"), "w") as fh:
        json.dump(metrics, fh, indent=1)
    with open(os.path.join(args.out_dir, "per_episode_test.csv"), "w",
              newline="") as fh:
        rows = [dict(target=t, **e) for t in TARGETS
                for e in evals[t]["per_episode"]]
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(args.out_dir, "dispatch_comparison.csv"), "w",
              newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(policy_rows[0].keys()))
        w.writeheader()
        w.writerows(policy_rows)
    if not args.no_plots:
        make_plots(args.out_dir, shipped, args.target, split, ev,
                   policy_rows, list(arms))

    # console summary ---------------------------------------------------------
    print("shipped model   %s" % shipped.model_id)
    print("episodes        train/val/test = %d/%d/%d" % (
        len(split["train"]), len(split["val"]), len(split["test"])))
    for t in TARGETS:
        e = evals[t]
        lo, hi = e["mae_difference_95ci_episode_bootstrap"]
        print("T03 [%-6s]    MAE ridge %.3f vs persistence %.3f (diff %+.3f, "
              "95%% CI %+.3f..%+.3f); RMSE %.3f vs %.3f; episodes better "
              "%d / worse %d; alpha %s" % (
                  t, e["ridge_mae"], e["persistence_mae"],
                  e["mae_difference_ridge_minus_persistence"], lo, hi,
                  e["ridge_rmse"], e["persistence_rmse"],
                  e["episodes_ridge_better"], e["episodes_ridge_worse"],
                  fits[t]["selected_alpha"]))
    print("latency p95     %.4f ms features+predict, %.4f ms full tick" % (
        metrics["latency_ms"]["features_plus_predict_p95"],
        metrics["latency_ms"]["full_tick_p95"]))
    for arm, a in policy_summary["arms"].items():
        extra = ""
        if "paired_diff_vs_reactive_sec" in a:
            extra = " | vs reactive %+.1f s (95%% CI %+.1f..%+.1f), lower in " \
                    "%d/%d" % (a["paired_diff_vs_reactive_sec"],
                               a["paired_diff_95ci_sec"][0],
                               a["paired_diff_95ci_sec"][1],
                               a["episodes_lower_wait_than_reactive"],
                               a["episodes_compared"])
        print("dispatch %-18s wait %.1f s, boarded %d, censored %d, "
              "dispatches %d%s" % (arm, a["mean_of_episode_mean_wait_sec"],
                                   a["total_boarded"], a["total_censored"],
                                   a["total_dispatches"], extra))
    print("pass criteria   %s" % metrics["pass_criteria"])
    return metrics


# -------------------------------------------------------------------- plots
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"
C_MODEL, C_BASE, C_ALT = "#2a78d6", "#eb6834", "#1baf7a"


def _style(ax, title, xlabel, ylabel):
    ax.set_title(title, loc="left", fontsize=11, color=INK)
    ax.set_xlabel(xlabel, color=INK2, fontsize=9)
    ax.set_ylabel(ylabel, color=INK2, fontsize=9)
    ax.tick_params(colors=INK2, labelsize=8)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def make_plots(out_dir, model, target, split, ev, policy_rows, arms):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # (a) example held-out episodes: forecast vs label
    picks = []
    for want in ("ramp_up", "peak", "noisy_threshold"):
        for s in sorted(split["test"]):
            if make_spec(s).profile == want:
                picks.append(s)
                break
    fig, axes = plt.subplots(len(picks), 1, figsize=(9, 2.7 * len(picks)),
                             sharex=True)
    axes = np.atleast_1d(axes)
    for ax, s in zip(axes, picks):
        res = run_episode(make_spec(s), model)
        label_at = {r[3]: r[1] for r in supervised_rows(res, HORIZON, target)}
        ts = [r["t"] for r in res.ticks if r["t"] + HORIZON <= 360]
        by_t = {r["t"]: r for r in res.ticks}

        def series(key):  # NaN breaks the line wherever valid=false
            return [by_t[t][key] if by_t[t]["valid"] else np.nan for t in ts]

        ax.axvspan(0, 59, color=GRID, alpha=0.7, linewidth=0)
        ax.step(ts, [label_at.get(t, np.nan) for t in ts], where="post",
                color=INK2, linewidth=1.5, label="actual (label) 30 s later")
        ax.plot(ts, series("baseline"), color=C_BASE, linewidth=1.6,
                label="persistence baseline")
        ax.plot(ts, series("predicted"), color=C_MODEL, linewidth=2.0,
                label="ridge forecast")
        _style(ax, "Test episode %d (%s)" % (s, make_spec(s).profile),
               "", "passengers")
        ax.text(2, 0.92, "warm-up\nvalid=false", fontsize=7, color=INK2,
                va="top", transform=ax.get_xaxis_transform())
        ax.set_xlim(0, 335)
    axes[-1].set_xlabel("forecast issued at t (s); gaps = invalid forecasts",
                        color=INK2, fontsize=9)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, frameon=False, fontsize=8, ncol=3,
               loc="upper center")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(os.path.join(out_dir, "forecast_examples.png"), dpi=150)
    plt.close(fig)

    # (b) per-episode MAE: ridge vs persistence (dumbbell)
    eps = sorted(ev["per_episode"], key=lambda e: e["persistence_mae"])
    fig, ax = plt.subplots(figsize=(8, 0.42 * len(eps) + 1.4))
    for i, e in enumerate(eps):
        ax.plot([e["persistence_mae"], e["ridge_mae"]], [i, i],
                color=GRID, linewidth=2, zorder=1)
        ax.scatter(e["persistence_mae"], i, s=46, color=C_BASE, zorder=2,
                   edgecolor="white", linewidth=1.5,
                   label="persistence" if i == 0 else None)
        ax.scatter(e["ridge_mae"], i, s=46, color=C_MODEL, zorder=3,
                   edgecolor="white", linewidth=1.5,
                   label="ridge" if i == 0 else None)
        if not e["ridge_better"]:
            ax.text(max(e["ridge_mae"], e["persistence_mae"]) + 0.04, i,
                    "ridge worse", va="center", fontsize=8, color=INK)
    ax.set_yticks(range(len(eps)))
    ax.set_yticklabels(["%d %s" % (e["seed"], e["profile"]) for e in eps],
                       fontsize=8)
    _style(ax, "Held-out test episodes: 30 s forecast MAE (%s target)"
           % target, "mean absolute error (passengers)", "")
    ax.grid(axis="x", color=GRID, linewidth=0.8)
    ax.grid(axis="y", visible=False)
    ax.legend(frameon=False, fontsize=8, loc="lower right")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "per_episode_mae.png"), dpi=150)
    plt.close(fig)

    # (c) paired waits: each predictive arm vs reactive
    pred_arms = [a for a in arms if a != "reactive"]
    fig, axes = plt.subplots(1, len(pred_arms),
                             figsize=(4.8 * len(pred_arms), 4.8))
    axes = np.atleast_1d(axes)
    hi = max(r[a + "_mean_wait_boarded"] or 0 for r in policy_rows
             for a in arms) * 1.05
    for ax, arm, col in zip(axes, pred_arms, (C_MODEL, C_ALT)):
        pr = [r for r in policy_rows if r[arm + "_mean_wait_boarded"]
              is not None and r["reactive_mean_wait_boarded"] is not None]
        ax.plot([0, hi], [0, hi], color=INK2, linewidth=1, linestyle="--")
        ax.scatter([r["reactive_mean_wait_boarded"] for r in pr],
                   [r[arm + "_mean_wait_boarded"] for r in pr], s=40,
                   color=col, edgecolor="white", linewidth=1.5, zorder=3)
        ax.text(hi * 0.04, hi * 0.93, "above line: longer wait than reactive",
                fontsize=7.5, color=INK2)
        ax.text(hi * 0.40, hi * 0.05, "below line: shorter wait",
                fontsize=7.5, color=INK2)
        ax.set_xlim(0, hi)
        ax.set_ylim(0, hi)
        _style(ax, "%s (n=%d seeds)" % (arm.replace("_", ", ") + " target",
                                        len(pr)),
               "reactive: mean wait per episode (s)",
               "predictive: mean wait per episode (s)")
        ax.grid(axis="x", color=GRID, linewidth=0.8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "dispatch_wait_paired.png"), dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()

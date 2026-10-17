"""demand_predictor: S2 thirty-second passenger-demand forecast (owner M1-B).

Subscribes  /hub/queue   smart_hub_interfaces/QueueState        reliable, depth 10
Publishes   /hub/demand  smart_hub_interfaces/DemandPrediction  1 Hz, reliable 10
Publishes   /hub/events  smart_hub_interfaces/HubEvent          on change, R100

Behaviour (assignment sections 9.1-9.3, handbook S2):
  * keeps 1 Hz history of valid queue counts (60 s), publishes valid=false
    during warm-up, after stale/invalid input, and while the 31 s feature
    window still contains a gap;
  * publishes the trained ridge forecast and the persistence baseline side by
    side with the model ID; falls back to persistence if the model file is
    missing or invalid (unless require_model is true);
  * header.stamp of a valid forecast is the observation time of the queue
    sample it was computed from, never the publication time;
  * invalid output uses predicted_count = baseline_count = -1.0 (an invalid
    forecast is never a zero).

All decision logic lives in smart_hub.demand (pure Python, unit-tested); this
file only wires it to ROS.
"""

import collections
import os
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from smart_hub_interfaces.msg import DemandPrediction, HubEvent, QueueState

from smart_hub.demand.estimator import DemandEstimator
from smart_hub.demand.models import PersistenceModel, load_model

NODE_NAME = "demand_predictor"
DEFAULT_MODEL_FILE = "demand_ridge_h30.json"


def _qos(depth):
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)


def _stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def default_model_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("smart_hub")
        return os.path.join(share, "models", DEFAULT_MODEL_FILE)
    except Exception:  # noqa: BLE001 - package not installed (dev checkout)
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(os.path.dirname(here), "models", DEFAULT_MODEL_FILE)


class DemandPredictor(Node):
    PARAMS = {
        # name: (default, description)
        "model_path": ("", "model JSON; empty = installed default"),
        "use_model": (True, "false = publish persistence baseline only"),
        "require_model": (False, "true = refuse to start without the model"),
        "prediction_horizon_sec": (30, "must equal the model horizon"),
        "history_sec": (60, "length of the 1 Hz history buffer"),
        "warmup_sec": (60, "valid samples needed before first forecast"),
        "reset_gap_sec": (60, "missing seconds that clear the history"),
        "stale_timeout_sec": (1.0, "queue age that makes input stale"),
        "future_tolerance_sec": (0.5, "max accepted future source stamp"),
        "queue_confidence_min": (0.5, "lower confidence = UNKNOWN"),
        "queue_zone_id": ("Q", "zone_id accepted from /hub/queue"),
        "publish_rate_hz": (1.0, "forecast publication rate"),
        "latency_report_sec": (30.0, "period of the latency log line"),
        "run_id": ("", "experiment run ID for /hub/events"),
        "scenario_id": ("", "scenario ID for /hub/events"),
        "queue_topic": ("/hub/queue", ""),
        "demand_topic": ("/hub/demand", ""),
        "events_topic": ("/hub/events", ""),
    }

    def __init__(self, **node_kwargs):
        # node_kwargs (e.g. parameter_overrides) are used by the node tests.
        super().__init__(NODE_NAME, **node_kwargs)
        for name, (default, desc) in self.PARAMS.items():
            self.declare_parameter(name, default,
                                   ParameterDescriptor(description=desc))
        p = {n: self.get_parameter(n).value for n in self.PARAMS}
        self._validate(p)
        self.p = p

        self.events = self.create_publisher(HubEvent, p["events_topic"],
                                            _qos(100))
        self.model, model_note = self._load_model(p)
        self.estimator = DemandEstimator(
            self.model,
            history_sec=int(p["history_sec"]),
            warmup_sec=int(p["warmup_sec"]),
            reset_gap_sec=int(p["reset_gap_sec"]),
            stale_timeout_sec=float(p["stale_timeout_sec"]),
            future_tolerance_sec=float(p["future_tolerance_sec"]),
            confidence_min=float(p["queue_confidence_min"]),
        )
        self._stamps = collections.OrderedDict()  # float -> original Time
        self._last_state = None
        self._tick_ms = collections.deque(maxlen=600)
        self._infer_ms = collections.deque(maxlen=600)
        self._last_latency_log = time.monotonic()
        self._wrong_zone = 0
        self.last_output = None

        self.pub = self.create_publisher(DemandPrediction, p["demand_topic"],
                                         _qos(10))
        self.sub = self.create_subscription(QueueState, p["queue_topic"],
                                            self.on_queue, _qos(10))
        self.timer = self.create_timer(1.0 / float(p["publish_rate_hz"]),
                                       self.on_tick)
        self.add_on_set_parameters_callback(self._reject_runtime_changes)

        self.get_logger().info(
            "started: model_id=%s (%s); parameters=%s"
            % (self.model.model_id, model_note,
               {k: v for k, v in sorted(p.items())}))

    # ------------------------------------------------------------ startup
    @staticmethod
    def _validate(p):
        def bad(msg):
            raise ValueError("invalid parameter: " + msg)
        if int(p["prediction_horizon_sec"]) != 30:
            bad("prediction_horizon_sec must be 30 for the shipped model")
        if not 31 <= int(p["history_sec"]) <= 600:
            bad("history_sec must be in [31, 600]")
        if not 31 <= int(p["warmup_sec"]) <= int(p["history_sec"]):
            bad("warmup_sec must be in [31, history_sec]")
        if not 1 <= int(p["reset_gap_sec"]) <= 3600:
            bad("reset_gap_sec must be in [1, 3600]")
        if not 0.1 <= float(p["stale_timeout_sec"]) <= 5.0:
            bad("stale_timeout_sec must be in [0.1, 5]")
        if not 0.0 <= float(p["future_tolerance_sec"]) <= 2.0:
            bad("future_tolerance_sec must be in [0, 2]")
        if not 0.0 <= float(p["queue_confidence_min"]) <= 1.0:
            bad("queue_confidence_min must be in [0, 1]")
        if abs(float(p["publish_rate_hz"]) - 1.0) > 1e-9:
            # Features are defined on one-second samples; another rate would
            # silently change their meaning.
            bad("publish_rate_hz must be 1.0 (features are per-second)")
        if float(p["latency_report_sec"]) <= 0:
            bad("latency_report_sec must be positive")

    def _load_model(self, p):
        if not p["use_model"]:
            return PersistenceModel(), "use_model=false"
        path = p["model_path"] or default_model_path()
        try:
            model = load_model(path)
            if model.horizon_sec != int(p["prediction_horizon_sec"]):
                raise ValueError("model horizon %d != parameter %d" % (
                    model.horizon_sec, int(p["prediction_horizon_sec"])))
        except Exception as exc:  # noqa: BLE001
            if p["require_model"]:
                raise
            self.get_logger().error(
                "model load failed (%s): %s -> persistence fallback"
                % (path, exc))
            self._event("MODEL_FALLBACK", str(exc)[:200],
                        PersistenceModel.model_id)
            return PersistenceModel(), "FALLBACK: " + str(exc)[:80]
        self._event("MODEL_LOADED", path, model.model_id)
        return model, path

    def _reject_runtime_changes(self, params):
        # Parameters are part of the experiment record: restart to change.
        # Only this node's own parameters are locked (use_sim_time etc. are
        # left to rclpy).
        names = ", ".join(x.name for x in params if x.name in self.PARAMS)
        if not names:
            return SetParametersResult(successful=True)
        return SetParametersResult(
            successful=False,
            reason="demand_predictor parameters are fixed per run (%s); "
                   "restart the node with new values" % names)

    # ----------------------------------------------------------- callbacks
    def on_queue(self, msg):
        if msg.zone_id and msg.zone_id != self.p["queue_zone_id"]:
            self._wrong_zone += 1
            return
        stamp = _stamp_to_sec(msg.header.stamp)
        result = self.estimator.on_queue(
            msg.count, msg.valid, msg.confidence, stamp,
            time.monotonic(), self._now())
        if result == "ACCEPTED":
            self._stamps[stamp] = msg.header.stamp
            while len(self._stamps) > 20:
                self._stamps.popitem(last=False)

    def on_tick(self):
        # A timer-callback exception would stop rclpy.spin and kill the node
        # (silencing /hub/demand). Log it loudly and keep the 1 Hz loop alive;
        # consumers already treat a missing/old forecast as invalid.
        try:
            self._tick()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("tick failed: %r" % (exc,),
                                    throttle_duration_sec=5.0)

    def _tick(self):
        t0 = time.perf_counter()
        pred = self.estimator.tick(time.monotonic(), self._now())

        out = DemandPrediction()
        out.header.frame_id = "map"
        if pred.source_stamp is not None and pred.source_stamp in self._stamps:
            out.header.stamp = self._stamps[pred.source_stamp]
        else:
            out.header.stamp = self.get_clock().now().to_msg()
        out.horizon_sec = int(pred.horizon_sec)
        out.predicted_count = float(pred.predicted_count)
        out.baseline_count = float(pred.baseline_count)
        out.model_id = pred.model_id
        out.valid = bool(pred.valid)
        self.pub.publish(out)
        self.last_output = out  # inspected by the node tests

        state = (pred.valid, pred.reason)
        if state != self._last_state:
            self._event("DEMAND_VALID" if pred.valid else "DEMAND_INVALID",
                        pred.reason, pred.model_id)
            # Two separate call sites: rclpy forbids changing the severity
            # of one logging call site between calls (raises ValueError).
            if pred.valid:
                self.get_logger().info("forecast valid (%s)" % pred.reason)
            else:
                self.get_logger().warning(
                    "forecast INVALID (%s)" % pred.reason)
            self._last_state = state

        if pred.inference_ms is not None:
            self._infer_ms.append(pred.inference_ms)
        self._tick_ms.append((time.perf_counter() - t0) * 1000.0)
        self._maybe_log_latency()

    # ------------------------------------------------------------- helpers
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _event(self, event, reason, correlation_id):
        msg = HubEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.run_id = str(self.p["run_id"])
        msg.scenario_id = str(self.p["scenario_id"])
        msg.entity_id = NODE_NAME
        msg.event = event
        msg.reason = reason
        msg.correlation_id = correlation_id
        self.events.publish(msg)

    def _maybe_log_latency(self):
        now = time.monotonic()
        if now - self._last_latency_log < float(self.p["latency_report_sec"]):
            return
        self._last_latency_log = now

        def p(xs, q):
            if not xs:
                return float("nan")
            s = sorted(xs)
            return s[min(len(s) - 1, int(q * len(s)))]
        self.get_logger().info(
            "latency ms: inference p50=%.3f p95=%.3f (n=%d); tick p95=%.3f; "
            "rejected inputs=%s; wrong zone=%d" % (
                p(self._infer_ms, 0.5), p(self._infer_ms, 0.95),
                len(self._infer_ms), p(self._tick_ms, 0.95),
                dict(self.estimator.rejections), self._wrong_zone))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DemandPredictor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

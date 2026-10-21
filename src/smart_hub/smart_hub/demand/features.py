"""Feature extraction and 1 Hz history buffer for the S2 demand forecaster.

The same functions are used offline (training/evaluation) and online (the
ROS node), so the model never sees a feature computed differently from the
one it was trained on.

Feature vector at second t (all from observed, valid 1 Hz queue counts):

    q_t             latest valid queue count
    mean_10s        mean of q_{t-9..t}                (assignment: q_{t-9:t})
    mean_30s        mean of q_{t-29..t}
    trend_10s       q_t - q_{t-10}                    (assignment: ten-second trend)
    arrivals_30s    sum of positive 1 s count changes over the last 30 s
    departures_30s  sum of negative 1 s count changes over the last 30 s (as a
                    positive number)

QueueState carries only a count, not individual track entries/exits, so
arrivals/departures are *count-delta proxies*: if one person joins and another
leaves inside the same second, the net change is zero and both events are
lost. This is the documented proxy permitted by assignment section 9.2.
"""

from collections import deque
from typing import Deque, List, Optional, Sequence

FEATURE_NAMES = (
    'q_t',
    'mean_10s',
    'mean_30s',
    'trend_10s',
    'arrivals_30s',
    'departures_30s',
)

# Samples needed to compute every feature: q_{t-30} .. q_t (31 samples gives
# 30 one-second differences for the arrival/departure proxies).
FEATURE_WINDOW = 31


def extract_features(window: Sequence[float]) -> List[float]:
    """Return the feature vector for the newest sample in ``window``.

    ``window`` holds consecutive valid 1 Hz counts, oldest first, and must
    contain at least FEATURE_WINDOW samples. Gaps are not allowed: callers
    must never pass interpolated or repeated stale values.
    """
    if len(window) < FEATURE_WINDOW:
        raise ValueError(
            'need %d consecutive valid samples, got %d'
            % (FEATURE_WINDOW, len(window))
        )
    w = [float(x) for x in window[-FEATURE_WINDOW:]]
    q_t = w[-1]
    mean_10 = sum(w[-10:]) / 10.0
    mean_30 = sum(w[-30:]) / 30.0
    trend_10 = w[-1] - w[-11]
    arrivals = 0.0
    departures = 0.0
    for prev, cur in zip(w[:-1], w[1:]):
        delta = cur - prev
        if delta > 0:
            arrivals += delta
        elif delta < 0:
            departures -= delta
    return [q_t, mean_10, mean_30, trend_10, arrivals, departures]


class DemandHistory:
    """Sliding 1 Hz history with explicit gaps, warm-up and reset rules.

    One call to :meth:`push` per 1 Hz tick: a count when a fresh valid queue
    observation exists for that tick, ``None`` when it does not. Missing
    seconds are stored as gaps and are never filled with old values.

    Readiness rules (see README, "Validity rules"):
      * WARMUP        fewer than ``warmup_sec`` valid samples since start/reset
      * STALE_INPUT   the newest tick had no fresh valid observation
      * GAP_RECOVERY  a gap occurred less than FEATURE_WINDOW samples ago, so
                      the feature window is not yet gap-free
      * a run of ``reset_gap_sec`` consecutive missing ticks clears the
        history entirely and the full warm-up starts again.
    """

    def __init__(self, history_sec: int = 60, warmup_sec: int = 60,
                 reset_gap_sec: int = 60):
        if warmup_sec < FEATURE_WINDOW:
            raise ValueError('warmup_sec must be >= %d' % FEATURE_WINDOW)
        if history_sec < FEATURE_WINDOW:
            raise ValueError('history_sec must be >= %d' % FEATURE_WINDOW)
        if reset_gap_sec < 1:
            raise ValueError('reset_gap_sec must be >= 1')
        self.history_sec = int(history_sec)
        self.warmup_sec = int(warmup_sec)
        self.reset_gap_sec = int(reset_gap_sec)
        self.reset()

    def reset(self) -> None:
        self._samples: Deque[Optional[float]] = deque(maxlen=self.history_sec)
        self.valid_total = 0      # valid samples since start / last reset
        self.tail_run = 0         # consecutive valid samples at the end
        self.missing_run = 0      # consecutive missing samples at the end

    def push(self, count: Optional[float]) -> None:
        if count is None:
            self._samples.append(None)
            self.tail_run = 0
            self.missing_run += 1
            if self.missing_run >= self.reset_gap_sec:
                self.reset()
            return
        self._samples.append(float(count))
        self.valid_total += 1
        self.tail_run += 1
        self.missing_run = 0

    @property
    def latest(self) -> Optional[float]:
        return self._samples[-1] if self._samples else None

    def status(self) -> str:
        """Return 'OK' or the reason the forecast must be invalid."""
        if not self._samples or self._samples[-1] is None:
            if self.valid_total < self.warmup_sec:
                return 'WARMUP'
            return 'STALE_INPUT'
        if self.valid_total < self.warmup_sec:
            return 'WARMUP'
        if self.tail_run < FEATURE_WINDOW:
            return 'GAP_RECOVERY'
        return 'OK'

    def feature_window(self) -> List[float]:
        """Newest FEATURE_WINDOW samples; only valid when status() == 'OK'."""
        if self.status() != 'OK':
            raise RuntimeError('history not ready: ' + self.status())
        return list(self._samples)[-FEATURE_WINDOW:]

"""ROS-independent core of demand_predictor.

The node feeds every /hub/queue message to :meth:`DemandEstimator.on_queue`
(5 Hz) and calls :meth:`DemandEstimator.tick` from its 1 Hz timer. Clocks are
passed in explicitly so the same object runs inside ROS, inside the offline
simulator and inside unit tests:

* ``receipt_mono`` / ``now_mono``: local monotonic time (time.monotonic())
  used for the receipt-age watchdog; immune to wall-clock jumps.
* ``source_stamp`` / ``now_ros``: ROS time in seconds; used to check the age
  of the observation itself (header.stamp) and to reject future stamps.
"""

from dataclasses import dataclass
import math
import time
from typing import List, Optional

from smart_hub.demand.features import DemandHistory, extract_features

INVALID_VALUE = -1.0  # published in predicted/baseline fields when unknown


@dataclass
class Prediction:
    valid: bool
    predicted_count: float
    baseline_count: float
    model_id: str
    horizon_sec: int
    reason: str
    source_stamp: Optional[float] = None
    features: Optional[List[float]] = None
    inference_ms: Optional[float] = None


@dataclass
class _Sample:
    count: float
    source_stamp: float
    receipt_mono: float


class DemandEstimator:
    def __init__(self, model, history_sec: int = 60, warmup_sec: int = 60,
                 reset_gap_sec: int = 60, stale_timeout_sec: float = 1.0,
                 future_tolerance_sec: float = 0.5,
                 confidence_min: float = 0.5):
        if not 0.0 < stale_timeout_sec <= 5.0:
            raise ValueError('stale_timeout_sec must be in (0, 5]')
        if not 0.0 <= future_tolerance_sec <= 2.0:
            raise ValueError('future_tolerance_sec must be in [0, 2]')
        if not 0.0 <= confidence_min <= 1.0:
            raise ValueError('confidence_min must be in [0, 1]')
        self.model = model
        self.history = DemandHistory(history_sec, warmup_sec, reset_gap_sec)
        self.stale_timeout_sec = float(stale_timeout_sec)
        self.future_tolerance_sec = float(future_tolerance_sec)
        self.confidence_min = float(confidence_min)
        self._pending: Optional[_Sample] = None
        self._last_stamp: Optional[float] = None
        self.rejections = {}

    # ------------------------------------------------------------------ input
    def on_queue(self, count, valid: bool, confidence: float,
                 source_stamp: float, receipt_mono: float,
                 now_ros: float) -> str:
        """Ingest one QueueState. Returns 'ACCEPTED' or a rejection reason.

        Any rejected or invalid message clears the pending sample, so the
        next 1 Hz tick sees no fresh valid observation (UNKNOWN, never zero).
        """
        reason = self._check(count, valid, confidence, source_stamp, now_ros)
        if reason != 'ACCEPTED':
            self.rejections[reason] = self.rejections.get(reason, 0) + 1
            if reason != 'FROZEN_STAMP':
                self._pending = None
            return reason
        self._last_stamp = source_stamp
        self._pending = _Sample(float(count), source_stamp, receipt_mono)
        return reason

    def _check(self, count, valid, confidence, source_stamp, now_ros) -> str:
        if not valid:
            return 'INVALID_FLAG'
        try:
            c = float(count)
            conf = float(confidence)
        except (TypeError, ValueError):
            return 'BAD_COUNT'
        if not math.isfinite(c) or c < 0:
            return 'BAD_COUNT'
        if not math.isfinite(conf) or conf < self.confidence_min:
            return 'LOW_CONFIDENCE'
        if source_stamp - now_ros > self.future_tolerance_sec:
            return 'FUTURE_STAMP'
        if self._last_stamp is not None and source_stamp <= self._last_stamp:
            # Same or older observation re-sent: not new evidence. The
            # pending sample is kept, but it still ages out normally.
            return 'FROZEN_STAMP'
        return 'ACCEPTED'

    # --------------------------------------------------------------- 1 Hz tick
    def tick(self, now_mono: float, now_ros: float) -> Prediction:
        sample = self._pending
        self._pending = None  # each observation is used for one tick at most
        fresh = (
            sample is not None
            and now_mono - sample.receipt_mono <= self.stale_timeout_sec
            and now_ros - sample.source_stamp <= self.stale_timeout_sec
        )
        self.history.push(sample.count if fresh else None)

        model_id = self.model.model_id
        horizon = int(getattr(self.model, 'horizon_sec', 30))
        status = self.history.status()
        baseline = sample.count if fresh else INVALID_VALUE
        stamp = sample.source_stamp if fresh else None
        if status != 'OK':
            return Prediction(False, INVALID_VALUE, baseline, model_id,
                              horizon, status, stamp)

        t0 = time.perf_counter()
        try:
            feats = extract_features(self.history.feature_window())
            pred = float(self.model.predict(feats))
            if not math.isfinite(pred):
                raise ValueError('non-finite prediction')
        except Exception:  # noqa: BLE001 - any model fault => invalid output
            return Prediction(False, INVALID_VALUE, baseline, model_id,
                              horizon, 'MODEL_ERROR', stamp)
        ms = (time.perf_counter() - t0) * 1000.0
        return Prediction(True, pred, baseline, model_id, horizon, 'OK',
                          stamp, feats, ms)

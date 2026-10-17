"""S2 dispatch decision rule (assignment section 9.3), ROS-independent.

hub_coordinator (M2) owns the real DockBus client. This class is used here
for the offline predictive-vs-reactive waiting-time comparison and is offered
to M2 as a tested reference for the coordinator.

Interpretation used (document it in the report; confirm with M2/instructor):
  * escalation starts after ``confirm`` consecutive usable samples >= high;
  * escalation clears after ``clear`` consecutive usable samples < low;
  * while escalated, a bus is requested only if no mission is outstanding,
    the cooldown since the previous request has elapsed and a bus is
    available. Escalation therefore persists across missions while the
    queue stays high, but never produces two outstanding goals.
  * predictive mode uses the forecast when valid; otherwise it falls back to
    the observed count (reason FALLBACK). Reactive mode always uses the
    observed count. A sample that is neither valid forecast nor fresh
    observation resets the confirmation counter.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class Decision:
    dispatch: bool
    reason: str          # PREDICTED, FALLBACK, OBSERVED, or a hold reason
    signal: Optional[float]
    escalated: bool


class DispatchPolicy:
    def __init__(self, mode: str = "predictive", high: float = 6.0,
                 low: float = 4.0, confirm: int = 3, clear: int = 5,
                 cooldown_sec: float = 30.0):
        if mode not in ("predictive", "reactive"):
            raise ValueError("mode must be predictive or reactive")
        if not low < high:
            raise ValueError("low threshold must be below high threshold")
        self.mode = mode
        self.high = float(high)
        self.low = float(low)
        self.confirm = int(confirm)
        self.clear = int(clear)
        self.cooldown_sec = float(cooldown_sec)
        self.escalated = False
        self.outstanding = False
        self.last_request: Optional[float] = None
        self._high_run = 0
        self._low_run = 0
        self._source = None
        self.dispatch_count = 0

    def update(self, now: float, forecast_valid: bool, forecast: float,
               observed_valid: bool, observed: float,
               bus_available: bool = True) -> Decision:
        if self.mode == "predictive" and forecast_valid:
            source, signal = "PREDICTED", float(forecast)
        elif observed_valid:
            source = "FALLBACK" if self.mode == "predictive" else "OBSERVED"
            signal = float(observed)
        else:
            source, signal = None, None

        if source != self._source:  # never mix sources inside one run count
            self._high_run = 0
            self._low_run = 0
            self._source = source

        if signal is None:
            return Decision(False, "NO_VALID_SIGNAL", None, self.escalated)

        self._high_run = self._high_run + 1 if signal >= self.high else 0
        self._low_run = self._low_run + 1 if signal < self.low else 0
        if self._high_run >= self.confirm:
            self.escalated = True
        if self._low_run >= self.clear:
            self.escalated = False

        if not self.escalated:
            return Decision(False, "BELOW_THRESHOLD", signal, False)
        if self.outstanding:
            return Decision(False, "MISSION_OUTSTANDING", signal, True)
        if (self.last_request is not None
                and now - self.last_request < self.cooldown_sec):
            return Decision(False, "COOLDOWN", signal, True)
        if not bus_available:
            return Decision(False, "BUS_UNAVAILABLE", signal, True)

        self.outstanding = True
        self.last_request = now
        self.dispatch_count += 1
        return Decision(True, source, signal, True)

    def on_mission_finished(self) -> None:
        """Call on any terminal DockBus result (succeeded/aborted/canceled)."""
        self.outstanding = False

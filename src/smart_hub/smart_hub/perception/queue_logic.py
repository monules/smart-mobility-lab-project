"""Queue state from tracked people: counting, crossing hold, freeze (M1-A, S1).

QueueLogic turns the tracker output into the QueueState fields:

* count            confirmed people whose foot point is inside queue Q;
* crossing_occupied any confirmed person inside crossing C, held true until
                   C has been clear for crossing_clear_sec (2.0 s, 8.2 step 6);
* valid            false when the camera feed is silent or its source stamp
                   has not advanced for stale_sec (1.0 s, 8.2 step 5): the old
                   count is never re-published as fresh;
* confidence       mean detector score of the counted people (1.0 if none).

While invalid, crossing_occupied is reported true: an unknown crossing must
deny entry (assignment 16.1), never look clear. Times are seconds supplied
by the caller (monotonic receipt time and the image stamp).
"""
from smart_hub.perception.tracker import Tracker
from smart_hub.perception.zones import CROSSING_C, point_in_polygon, QUEUE_Q


class QueueLogic:
    def __init__(self, stale_sec=1.0, crossing_clear_sec=2.0, tracker=None):
        self.stale_sec = float(stale_sec)
        self.crossing_clear_sec = float(crossing_clear_sec)
        self.tracker = tracker or Tracker()
        self.last_rx = None            # monotonic receipt time of the last frame
        self.last_stamp = None         # source stamp of the last frame (s)
        self.stamp_changed_at = None   # receipt time when the stamp last advanced
        self.crossing_seen_at = None   # last time someone was inside C
        self.scores = {}               # track id -> latest detector score
        self.events = []               # ('QUEUE_VALID' | 'QUEUE_INVALID', reason)
        self._was_valid = None

    def on_frame(self, people, stamp, now):
        """people: list of (x, y, score) foot points in metres for one frame."""
        self.last_rx = now
        if self.last_stamp is None or stamp > self.last_stamp:
            self.last_stamp = stamp
            self.stamp_changed_at = now
        else:
            return                     # repeated/old frame: a frozen source
        self.tracker.update([(x, y) for x, y, _ in people], now)
        for tr in self.tracker.tracks:
            if tr.last_seen == now:
                best = min(people, key=lambda p: (p[0] - tr.x) ** 2 + (p[1] - tr.y) ** 2)
                self.scores[tr.track_id] = best[2]
        if any(point_in_polygon(tr.x, tr.y, CROSSING_C) for tr in self.tracker.counted(now)):
            self.crossing_seen_at = now

    def validity(self, now):
        if self.last_rx is None:
            return False, 'NO_IMAGE'
        if now - self.last_rx > self.stale_sec:
            return False, 'FEED_SILENT'
        if now - self.stamp_changed_at > self.stale_sec:
            return False, 'FROZEN_STAMP'
        return True, 'OK'

    def state(self, now):
        """Return dict(count, confidence, valid, crossing_occupied, stamp, reason)."""
        valid, reason = self.validity(now)
        if valid != self._was_valid:
            self.events.append(('QUEUE_VALID' if valid else 'QUEUE_INVALID', reason))
            self._was_valid = valid
        if not valid:
            return {'count': 0, 'confidence': 0.0, 'valid': False,
                    'crossing_occupied': True, 'stamp': None, 'reason': reason}
        counted = self.tracker.counted(now)
        in_queue = [tr for tr in counted if point_in_polygon(tr.x, tr.y, QUEUE_Q)]
        scores = [self.scores.get(tr.track_id, 1.0) for tr in in_queue]
        crossing = (self.crossing_seen_at is not None
                    and now - self.crossing_seen_at < self.crossing_clear_sec)
        return {'count': len(in_queue),
                'confidence': min(1.0, sum(scores) / len(scores)) if scores else 1.0,
                'valid': True, 'crossing_occupied': crossing, 'stamp': self.last_stamp,
                'reason': reason}

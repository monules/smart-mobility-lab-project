"""Frame-to-frame person tracking on the floor plane (M1-A, 8.2 steps 2 and 4).

Tracks live in map coordinates (metres), so association distances mean the
same anywhere in the image. Rules (assignment 8.2 step 4, role doc step 5):

* a new detection becomes a *tentative* track; it is *confirmed* once it has
  been seen for confirm_sec (0.4 s debounce), so a one-frame false
  detection never counts;
* a confirmed track keeps counting while it was seen within hold_sec
  (0.4 s debounce on disappearance);
* a track survives forget_sec (0.5 s) without detections, so a short
  occlusion (T02: 0.3 s) does not create a new passenger: when the person
  reappears near the old position, the same id continues.

Re-detecting the same people every frame updates tracks and never adds
passengers, so repeated frames cannot inflate the count (T01).
"""
from dataclasses import dataclass
import math


@dataclass
class Track:
    track_id: int
    x: float
    y: float
    first_seen: float
    last_seen: float
    confirmed: bool = False


class Tracker:
    def __init__(self, match_distance=0.6, confirm_sec=0.4, hold_sec=0.4, forget_sec=0.5):
        self.match_distance = float(match_distance)
        self.confirm_sec, self.hold_sec = float(confirm_sec), float(hold_sec)
        self.forget_sec = float(forget_sec)
        self.tracks = []
        self.next_id = 1
        self.events = []       # ('TRACK_CONFIRMED' | 'TRACK_LOST', track_id)

    def update(self, points, t):
        """Associate this frame's floor points (x, y) with tracks at time t."""
        # Greedy nearest-neighbour association: closest pairs first, each
        # track and each point used once, only within match_distance.
        pairs = sorted((math.hypot(tr.x - px, tr.y - py), i, j)
                       for i, tr in enumerate(self.tracks)
                       for j, (px, py) in enumerate(points))
        used_tracks, used_points = set(), set()
        for dist, i, j in pairs:
            if dist > self.match_distance or i in used_tracks or j in used_points:
                continue
            tr = self.tracks[i]
            tr.x, tr.y = points[j]
            tr.last_seen = t
            used_tracks.add(i)
            used_points.add(j)
        for j, (px, py) in enumerate(points):
            if j not in used_points:
                self.tracks.append(Track(self.next_id, px, py, t, t))
                self.next_id += 1
        self._age(t)

    def _age(self, t):
        kept = []
        for tr in self.tracks:
            if t - tr.last_seen > self.forget_sec:
                if tr.confirmed:
                    self.events.append(('TRACK_LOST', tr.track_id))
                continue
            if not tr.confirmed and tr.last_seen - tr.first_seen >= self.confirm_sec:
                tr.confirmed = True
                self.events.append(('TRACK_CONFIRMED', tr.track_id))
            kept.append(tr)
        self.tracks = kept

    def counted(self, t):
        """Return confirmed tracks seen within hold_sec: the people that count now."""
        return [tr for tr in self.tracks
                if tr.confirmed and t - tr.last_seen <= self.hold_sec]

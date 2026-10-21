"""Synthetic overhead camera for the simulator (M5).

A pinhole camera south of the map looks down at queue Q and crossing C and
renders every visible pedestrian as a simple upright figure, so
passenger_detector (M1-A) has a /camera/image_raw stream to work on before a
real camera exists (assignment 1.3: synthetic detections; 8.1).

Projection is plain numpy (no ROS, unit-testable); drawing uses OpenCV.
The detector never reads this file's parameters: it calibrates itself from
four ground points, as it would with a real camera (assignment 8.2, step 3).
"""
import numpy as np

WIDTH, HEIGHT = 640, 480
FOCAL_PX = 700.0
POSITION = (4.2, -3.5, 8.0)     # metres: south of the map, 8 m up
TARGET = (4.2, 4.2, 0.0)        # point on the ground at the image centre
PERSON_HEIGHT = 1.75            # m
BACKGROUND = (205, 205, 205)    # BGR, light grey floor
FLOOR_LINE = (165, 165, 165)    # zone outlines painted on the floor
PERSON_COLOR = (70, 45, 30)     # BGR, dark clothing


class PinholeCamera:
    def __init__(self, position=POSITION, target=TARGET, focal_px=FOCAL_PX,
                 width=WIDTH, height=HEIGHT):
        self.pos = np.asarray(position, float)
        fwd = np.asarray(target, float) - self.pos
        self.fwd = fwd / np.linalg.norm(fwd)
        right = np.cross(self.fwd, [0.0, 0.0, 1.0])
        self.right = right / np.linalg.norm(right)        # image +u
        self.down = np.cross(self.fwd, self.right)         # image +v
        self.f, self.w, self.h = float(focal_px), int(width), int(height)

    def project(self, x, y, z=0.0):
        """Map point (metres) -> pixel (u, v), or None if behind the camera."""
        d = np.array([x, y, z], float) - self.pos
        depth = d @ self.fwd
        if depth <= 1e-6:
            return None
        return (self.w / 2 + self.f * (d @ self.right) / depth,
                self.h / 2 + self.f * (d @ self.down) / depth)

    def render(self, people, floor_rects=()):
        """BGR image with floor zone outlines and one figure per (x, y).

        Figures are drawn far-to-near so nearer people overlap farther ones,
        like a real camera.
        """
        import cv2
        img = np.zeros((self.h, self.w, 3), np.uint8)
        img[:] = BACKGROUND
        for xmin, ymin, xmax, ymax in floor_rects:
            corners = [self.project(x, y) for x, y in
                       ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax))]
            if None not in corners:
                pts = np.array(corners, np.int32).reshape(-1, 1, 2)
                cv2.polylines(img, [pts], True, FLOOR_LINE, 2)
        for x, y in sorted(people, key=lambda p: -p[1]):     # far (north) first
            self._draw_person(img, x, y, cv2)
        return img

    def _draw_person(self, img, x, y, cv2):
        foot, head = self.project(x, y, 0.0), self.project(x, y, PERSON_HEIGHT)
        if foot is None or head is None:
            return
        fu, fv = foot
        h = fv - head[1]
        w = 0.30 * h
        c = PERSON_COLOR

        def box(x0, y0, x1, y1):
            cv2.rectangle(img, (int(fu + x0 * w), int(fv - y1 * h)),
                          (int(fu + x1 * w), int(fv - y0 * h)), c, -1)
        box(-0.35, 0.0, -0.05, 0.47)                 # left leg
        box(0.05, 0.0, 0.35, 0.47)                   # right leg
        box(-0.50, 0.45, 0.50, 0.82)                 # torso and arms
        cv2.circle(img, (int(fu), int(fv - 0.9 * h)), max(1, int(0.1 * h)), c, -1)

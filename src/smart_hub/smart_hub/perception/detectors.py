"""Person detectors returning bounding boxes and foot points (M1-A, 8.2 step 1).

Two backends, chosen by the passenger_detector parameter ``detector``:

* ``blob`` — documented simple baseline for the simulator camera: people are
  the only dark objects on a light floor, so dark connected regions above a
  minimum area are people. Deterministic; used for S1, T01 and T02.
* ``hog`` — OpenCV's pretrained HOG + linear SVM people detector
  (``HOGDescriptor_getDefaultPeopleDetector``, Dalal & Triggs), shipped
  inside OpenCV, for the real camera/video demonstration. Its window is
  64x128 px, so small images are upscaled first.

Each detection is (x, y, w, h, score) in the input image's pixels; foot()
gives the ground-contact pixel used for the homography (8.2 step 2).
"""
import cv2
import numpy as np


class BlobDetector:
    name = 'blob-dark-v1'

    def __init__(self, max_gray=110, min_area=150):
        self.max_gray, self.min_area = int(max_gray), int(min_area)
        self.foot_offset = 0.0           # box bottom is the feet

    def detect(self, bgr):
        mask = cv2.inRange(bgr, (0, 0, 0), (self.max_gray,) * 3)
        n, _, stats, _ = cv2.connectedComponentsWithStats(mask)
        return [(int(x), int(y), int(w), int(h), 1.0)
                for x, y, w, h, area in stats[1:] if area >= self.min_area]


class HogDetector:
    name = 'opencv-hog-default-people'

    def __init__(self, threshold=0.5, upscale=1.5, stride=8, foot_offset=0.15):
        self.threshold, self.upscale = float(threshold), float(upscale)
        self.stride = int(stride)
        # HOG boxes include a margin below the feet (about 15% of the box in
        # the 64x128 training window), so the foot is above the box bottom.
        self.foot_offset = float(foot_offset)
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def detect(self, bgr):
        img = bgr if self.upscale == 1.0 else cv2.resize(
            bgr, None, fx=self.upscale, fy=self.upscale)
        rects, weights = self.hog.detectMultiScale(
            img, winStride=(self.stride, self.stride), padding=(8, 8), scale=1.05)
        out = []
        for (x, y, w, h), score in zip(rects, np.ravel(weights)):
            if score >= self.threshold:
                s = self.upscale
                out.append((int(x / s), int(y / s), int(w / s), int(h / s), float(score)))
        return out


def make_detector(name, **params):
    if name == 'blob':
        return BlobDetector(params.get('max_gray', 110), params.get('min_area', 150))
    if name == 'hog':
        return HogDetector(params.get('threshold', 0.5), params.get('upscale', 1.5),
                           params.get('stride', 8), params.get('foot_offset', 0.15))
    raise ValueError('detector must be blob or hog, not %r' % (name,))


def foot(det, foot_offset):
    """Ground-contact pixel of a detection: bottom centre of the box."""
    x, y, w, h, _ = det
    return x + w / 2.0, y + h * (1.0 - foot_offset)

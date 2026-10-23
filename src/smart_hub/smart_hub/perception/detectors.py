"""Person detectors returning bounding boxes and foot points (M1-A, 8.2 step 1).

Two backends, chosen by the passenger_detector parameter ``detector``:

* ``blob`` — documented simple baseline for the simulator camera: people are
  the only dark objects on a light floor, so dark connected regions above a
  minimum area are people. Deterministic; used for S1, T01 and T02.
* ``hog`` — OpenCV's pretrained HOG + linear SVM people detector
  (``HOGDescriptor_getDefaultPeopleDetector``, Dalal & Triggs), shipped
  inside OpenCV. Its window is 64x128 px, so small images are upscaled first.
  It expects whole upright people; on the lab video (heads out of frame,
  camera looking down) it misses most of them.
* ``yolo`` — pretrained YOLOv4-tiny (Darknet, COCO, class 0 = person) run by
  OpenCV's DNN module. The weights are downloaded once into
  ``models/yolov4-tiny/`` (README_M1A.md says how); they are not in git.
  Used for the real lab video: it finds people without heads or seen from
  above. Score threshold 0.5, non-maximum suppression 0.4.

Each detection is (x, y, w, h, score) in the input image's pixels; foot()
gives the ground-contact pixel used for the homography (8.2 step 2).
"""
import os

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


class YoloDetector:
    name = 'opencv-dnn-yolov4-tiny-coco'

    def __init__(self, model_dir='models/yolov4-tiny', threshold=0.5, nms=0.4,
                 size=(608, 352), foot_offset=0.0):
        cfg = os.path.join(model_dir, 'yolov4-tiny.cfg')
        weights = os.path.join(model_dir, 'yolov4-tiny.weights')
        if not (os.path.isfile(cfg) and os.path.isfile(weights)):
            raise ValueError('YOLOv4-tiny files missing in %r (see README_M1A.md)' % model_dir)
        self.threshold, self.nms = float(threshold), float(nms)
        self.foot_offset = float(foot_offset)        # boxes end at the shoes
        self.model = cv2.dnn_DetectionModel(cv2.dnn.readNetFromDarknet(cfg, weights))
        self.model.setInputParams(size=tuple(size), scale=1 / 255.0, swapRB=True)

    def detect(self, bgr):
        classes, scores, boxes = self.model.detect(bgr, confThreshold=self.threshold,
                                                   nmsThreshold=self.nms)
        return [(int(x), int(y), int(w), int(h), float(s))
                for c, s, (x, y, w, h) in zip(np.ravel(classes), np.ravel(scores), boxes)
                if int(c) == 0]


def make_detector(name, **params):
    if name == 'blob':
        return BlobDetector(params.get('max_gray', 110), params.get('min_area', 150))
    if name == 'hog':
        return HogDetector(params.get('threshold', 0.5), params.get('upscale', 1.5),
                           params.get('stride', 8), params.get('foot_offset', 0.15))
    if name == 'yolo':
        return YoloDetector(params.get('model_dir', os.environ.get(
            'SMART_HUB_YOLO_DIR', 'models/yolov4-tiny')), params.get('threshold', 0.5),
            params.get('nms', 0.4))
    raise ValueError('detector must be blob, hog or yolo, not %r' % (name,))


def foot(det, foot_offset):
    """Ground-contact pixel of a detection: bottom centre of the box."""
    x, y, w, h, _ = det
    return x + w / 2.0, y + h * (1.0 - foot_offset)

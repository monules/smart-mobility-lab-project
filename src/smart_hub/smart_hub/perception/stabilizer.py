"""Hand-held / sagging camera compensation for recorded video (M1-A).

The lab phone drifted up to ~67 px during the 70 s take, so tape corners
picked on the first frame no longer line up later. FloorStabilizer matches
ORB features of the static floor (the lower part of the picture, where people
rarely stand) between the reference frame and each new frame, fits a
homography with RANSAC, and maps detection pixels back into the reference
frame before the floor homography is applied. If too few features match, the
last good transform is kept and the frame is marked as not aligned.
"""
import cv2
import numpy as np


class FloorStabilizer:
    def __init__(self, reference_bgr, roi_top=0.35, features=3000, min_inliers=40):
        self.min_inliers = int(min_inliers)
        self.orb = cv2.ORB_create(int(features))
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        gray = cv2.cvtColor(reference_bgr, cv2.COLOR_BGR2GRAY)
        self.mask = np.zeros_like(gray)
        self.mask[int(roi_top * gray.shape[0]):, :] = 255
        self.ref_kp, self.ref_des = self.orb.detectAndCompute(gray, self.mask)
        self.H = np.eye(3)                       # frame -> reference
        self.inliers = 0

    def update(self, bgr):
        """Estimate frame -> reference; return True if this frame was aligned."""
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        kp, des = self.orb.detectAndCompute(gray, self.mask)
        if des is None or self.ref_des is None:
            return False
        matches = sorted(self.matcher.match(des, self.ref_des), key=lambda m: m.distance)[:500]
        if len(matches) < self.min_inliers:
            return False
        src = np.float32([kp[m.queryIdx].pt for m in matches])
        dst = np.float32([self.ref_kp[m.trainIdx].pt for m in matches])
        H, inl = cv2.findHomography(src, dst, cv2.RANSAC, 2.0)
        if H is None or int(inl.sum()) < self.min_inliers:
            return False
        self.H, self.inliers = H, int(inl.sum())
        return True

    def to_reference(self, u, v):
        x, y, w = self.H @ np.array([u, v, 1.0])
        return float(x / w), float(y / w)

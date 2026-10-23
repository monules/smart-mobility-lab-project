"""M1-A real-video helpers: drift stabiliser and the YOLO detector loader."""
import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')
from smart_hub.perception.detectors import make_detector  # noqa: E402
from smart_hub.perception.stabilizer import FloorStabilizer  # noqa: E402


def textured_floor(seed=3):
    rng = np.random.default_rng(seed)
    img = (rng.random((540, 960)) * 255).astype(np.uint8)
    img = cv2.GaussianBlur(img, (3, 3), 0)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def test_stabilizer_maps_a_drifted_frame_back_to_the_reference():
    ref = textured_floor()
    shift = np.float32([[1, 0, 14], [0, 1, -9]])             # camera drifted 14 px, -9 px
    moved = cv2.warpAffine(ref, shift, (ref.shape[1], ref.shape[0]),
                           borderMode=cv2.BORDER_REFLECT)
    st = FloorStabilizer(ref)
    assert st.update(moved)
    u, v = st.to_reference(500 + 14, 400 - 9)               # a foot seen in the moved frame
    assert abs(u - 500) < 1.0 and abs(v - 400) < 1.0


def test_stabilizer_keeps_the_last_transform_without_features():
    st = FloorStabilizer(textured_floor())
    assert not st.update(np.zeros((540, 960, 3), np.uint8))  # nothing to match
    assert np.allclose(st.H, np.eye(3))


def test_yolo_detector_explains_missing_weights(tmp_path):
    with pytest.raises(ValueError, match='README_M1A'):
        make_detector('yolo', model_dir=str(tmp_path))
    with pytest.raises(ValueError, match='blob, hog or yolo'):
        make_detector('ssd')

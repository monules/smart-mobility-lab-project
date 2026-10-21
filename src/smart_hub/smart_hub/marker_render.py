"""Draw dashboard markers as a top-down picture (M5, pure Python + OpenCV).

RViz needs a screen, and the dev container has none. This renderer turns the
same marker content that /hub/markers carries into a PNG, so the dashboard
can be seen (and put in the report) from any computer. Input: marker dicts
as produced by MarkerCore.build(), or converted back from
visualization_msgs/Marker by dashboard_snapshot.
"""
import numpy as np

PX_PER_M = 70
MARGIN = 30          # px around the 12 m x 8 m site
HEADER = 80          # px above the site for the status text (y > 8 m)
WIDTH = int(2 * MARGIN + 12 * PX_PER_M)
HEIGHT = int(HEADER + MARGIN + 8 * PX_PER_M)
BACKGROUND = (30, 30, 30)


def to_px(x, y):
    """Map metres (map frame, y up) to pixels (image, v down)."""
    return int(round(MARGIN + x * PX_PER_M)), int(round(HEADER + (8.0 - y) * PX_PER_M))


def bgr(color):
    r, g, b = color[:3]
    return int(b * 255), int(g * 255), int(r * 255)


def _blend(img, draw, alpha):
    """Draw with transparency: draw on a copy, then mix it in."""
    layer = img.copy()
    draw(layer)
    np.copyto(img, (alpha * layer + (1 - alpha) * img).astype(np.uint8))


def render(markers, title=''):
    """Return a BGR image of the markers, drawn in order of their height (z)."""
    import cv2
    img = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
    img[:] = BACKGROUND
    for m in sorted(markers, key=lambda m: m['pos'][2]):
        kind, color = m['type'], m['color']
        alpha = max(0.15, min(1.0, color[3]))
        x, y = m['pos'][0], m['pos'][1]
        if kind == 'cube':
            w, h = m['scale'][0], m['scale'][1]
            p0, p1 = to_px(x - w / 2, y + h / 2), to_px(x + w / 2, y - h / 2)
            _blend(img, lambda im: cv2.rectangle(im, p0, p1, bgr(color), -1), alpha)
        elif kind == 'cylinder':
            radius = max(2, int(round(m['scale'][0] / 2 * PX_PER_M)))
            _blend(img, lambda im: cv2.circle(im, to_px(x, y), radius, bgr(color), -1), alpha)
        elif kind in ('line_list', 'line_strip'):
            pts = [to_px(px + x, py + y) for px, py in m.get('points', [])]
            width = max(1, int(round(m['scale'][0] * PX_PER_M / 2)))
            if kind == 'line_list':
                segments = list(zip(pts[0::2], pts[1::2]))
            else:
                segments = list(zip(pts[:-1], pts[1:]))
            for a, b in segments:
                cv2.line(img, a, b, bgr(color), width, cv2.LINE_AA)
        elif kind == 'text':
            size = max(0.35, m['scale'][2] * 1.6)
            u, v = to_px(x, y)
            text = m.get('text', '')
            (tw, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, size, 1)
            cv2.putText(img, text, (max(2, u - tw // 2), v), cv2.FONT_HERSHEY_SIMPLEX, size,
                        bgr(color), 1, cv2.LINE_AA)
    if title:
        cv2.putText(img, title, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1,
                    cv2.LINE_AA)
    return img

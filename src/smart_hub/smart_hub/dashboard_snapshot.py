"""dashboard_snapshot (M5 tool): save /hub/markers as pictures and a video.

Subscribes  /hub/markers  visualization_msgs/MarkerArray  R10
Writes      <output_dir>/dashboard/latest.png         (always the newest view)
            <output_dir>/dashboard/frame_<t>.png      (every period_sec)
            <output_dir>/dashboard/dashboard.mp4      (if video is true)

A read-only viewer for computers without RViz (e.g. the Docker container on
a Mac). It never publishes anything.
"""
import os
import time

import rclpy
from rclpy.node import Node
from smart_hub.marker_render import render
from smart_hub.ros_common import declare, R10
from visualization_msgs.msg import Marker, MarkerArray

TYPE_NAMES = {Marker.CUBE: 'cube', Marker.CYLINDER: 'cylinder',
              Marker.LINE_LIST: 'line_list', Marker.LINE_STRIP: 'line_strip',
              Marker.TEXT_VIEW_FACING: 'text'}


def marker_to_dict(m):
    return {'ns': m.ns, 'id': m.id, 'type': TYPE_NAMES.get(m.type, 'unknown'),
            'pos': (m.pose.position.x, m.pose.position.y, m.pose.position.z),
            'scale': (m.scale.x, m.scale.y, m.scale.z),
            'color': (m.color.r, m.color.g, m.color.b, m.color.a),
            'points': [(p.x, p.y) for p in m.points], 'text': m.text}


class DashboardSnapshot(Node):
    def __init__(self, **kwargs):
        super().__init__('dashboard_snapshot', **kwargs)
        p = declare(self, {'output_dir': 'output', 'period_sec': 1.0, 'video': True,
                           'video_fps': 5.0})
        self.dir = os.path.join(p['output_dir'], 'dashboard')
        os.makedirs(self.dir, exist_ok=True)
        self.period = float(p['period_sec'])
        self.fps = float(p['video_fps'])
        self.want_video = bool(p['video'])
        self.writer = None
        self.t0 = time.monotonic()
        self.last_png = -1e9
        self.latest = None
        self.create_subscription(MarkerArray, '/hub/markers', self.on_markers, R10)
        self.create_timer(1.0 / self.fps, self.on_frame)

    def on_markers(self, msg):
        self.latest = [marker_to_dict(m) for m in msg.markers if m.type in TYPE_NAMES]

    def on_frame(self):
        if self.latest is None:
            return
        import cv2
        t = time.monotonic() - self.t0
        img = render(self.latest, 't = %.1f s' % t)
        cv2.imwrite(os.path.join(self.dir, 'latest.png'), img)
        if t - self.last_png >= self.period:
            self.last_png = t
            cv2.imwrite(os.path.join(self.dir, 'frame_%06.1f.png' % t), img)
        if self.want_video:
            if self.writer is None:
                self.writer = cv2.VideoWriter(os.path.join(self.dir, 'dashboard.mp4'),
                                              cv2.VideoWriter_fourcc(*'mp4v'), self.fps,
                                              (img.shape[1], img.shape[0]))
            self.writer.write(img)

    def close(self):
        if self.writer is not None:
            self.writer.release()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardSnapshot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()                 # finish the video file even on Ctrl+C
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

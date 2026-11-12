"""control_center (M5): web UI to choose, run and watch any hub task.

    docker/hub.sh ui        then open http://localhost:8080

Subscribes  /hub/vehicles     VehicleStateArray  R10   agents and obstacle O
            /sim/truth        VehicleStateArray  R10   pedestrians only (display)
            /hub/safety       SafetyState        R1T   latch, reason
            /hub/estop        Bool               R1T   e-stop input
            /crossing/state   CrossingState      R1T   owner of C
            /parking/state    ParkingState       R1T   slots P1-P3
            /hub/queue        QueueState         R10   count in Q, C occupied
            /hub/demand       DemandPrediction   R10   30 s future-queue forecast
            /hub/demand_unserved  DemandPrediction  R10  30 s unserved-demand forecast
            /traffic/edges    EdgeStateArray     R10   road labels
            /robot/route,
            /bus/route        nav_msgs/Path      R1T   planned routes
            /hub/events       HubEvent           R100  event log
Publishes   nothing.

The node has no control authority over the hub: it never publishes, calls a
service or joins the reset handshake. It starts tasks the same way a person
would in a terminal (ros2 launch / ros2 run in a subprocess, see runs.py),
so every run produces the same evidence files as docker/hub.sh. The HTTP
server runs in a background thread and only serves this package's page and
a small JSON API; option values are checked against the catalog before
anything is started.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import threading
import time
from urllib.parse import parse_qs, urlparse

from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from smart_hub.control_center import catalog, intent
from smart_hub.control_center.live_state import LiveState, static_map
from smart_hub.control_center.runs import RunManager
from smart_hub.ros_common import declare, R10, R100, R1T, RESET_REQUEST
from smart_hub_interfaces.msg import (CrossingState, DemandPrediction, EdgeStateArray,
                                      HubEvent, ParkingState, QueueState, SafetyState,
                                      VehicleStateArray)
from std_msgs.msg import Bool

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web')
STATIC = {'/': ('index.html', 'text/html; charset=utf-8'),
          '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
          '/style.css': ('style.css', 'text/css; charset=utf-8')}
MAX_BODY = 64 * 1024
CSP = ("default-src 'self'; img-src 'self' data:; style-src 'self'; script-src 'self'; "
       "object-src 'none'; base-uri 'none'; frame-ancestors 'none'")


class ControlCenter(Node):
    def __init__(self, **kwargs):
        super().__init__('control_center', **kwargs)
        p = declare(self, {'port': 8080, 'host': '0.0.0.0', 'root': os.getcwd()})
        self.live = LiveState()
        self.runs = RunManager(p['root'])
        self.root = p['root']
        self._last_active = None
        self.create_subscription(VehicleStateArray, '/hub/vehicles', self.on_vehicles, R10)
        self.create_subscription(VehicleStateArray, '/sim/truth', self.on_truth, R10)
        self.create_subscription(SafetyState, '/hub/safety', self.on_safety, R1T)
        self.create_subscription(Bool, '/hub/estop', self.on_estop, R1T)
        self.create_subscription(CrossingState, '/crossing/state', self.on_crossing, R1T)
        self.create_subscription(ParkingState, '/parking/state', self.on_parking, R1T)
        self.create_subscription(QueueState, '/hub/queue', self.on_queue, R10)
        self.create_subscription(DemandPrediction, '/hub/demand', self.on_demand, R10)
        self.create_subscription(DemandPrediction, '/hub/demand_unserved',
                                 lambda m: self.on_demand(m, 'demand_unserved'), R10)
        self.create_subscription(EdgeStateArray, '/traffic/edges', self.on_edges, R10)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100)
        for agent in ('robot', 'bus'):
            self.create_subscription(Path, '/%s/route' % agent,
                                     lambda m, a=agent: self.on_route(a, m), R1T)
        self.server = ThreadingHTTPServer((p['host'], int(p['port'])), make_handler(self))
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, name='cc-http', daemon=True).start()
        self.get_logger().info('Control Center on http://localhost:%d' % int(p['port']))

    # ---------------------------------------------------------- subscriptions
    def on_vehicles(self, msg):
        self.live.on_vehicles([{'id': v.vehicle_id, 'kind': v.kind, 'x': round(v.pose.x, 3),
                                'y': round(v.pose.y, 3), 'theta': round(v.pose.theta, 3),
                                'speed': round(v.speed_mps, 3), 'edge': v.edge_id,
                                'valid': v.valid} for v in msg.vehicles], time.monotonic())

    def on_truth(self, msg):
        self.live.on_pedestrians([{'id': v.vehicle_id, 'x': round(v.pose.x, 3),
                                   'y': round(v.pose.y, 3)}
                                  for v in msg.vehicles if v.kind == 'pedestrian'],
                                 time.monotonic())

    def on_safety(self, msg):
        self.live.on_safety(msg.stop_latched, msg.motion_permitted, msg.reason,
                            time.monotonic())

    def on_estop(self, msg):
        self.live.on_estop(msg.data, time.monotonic())

    def on_crossing(self, msg):
        self.live.on_crossing(msg.state, msg.owner_id, msg.pedestrian_present, msg.valid,
                              time.monotonic())

    def on_parking(self, msg):
        self.live.on_parking({s.slot_id: {'state': s.state, 'vehicle': s.vehicle_id}
                              for s in msg.slots}, time.monotonic())

    def on_queue(self, msg):
        self.live.on_queue(msg.count, msg.confidence, msg.crossing_occupied, msg.valid,
                           time.monotonic())

    def on_demand(self, msg, key='demand'):
        self.live.on_demand(msg.predicted_count, msg.baseline_count, msg.horizon_sec,
                            msg.model_id, msg.valid, time.monotonic(), key)

    def on_edges(self, msg):
        self.live.on_edges({e.edge_id: {'blocked': e.blocked, 'valid': e.valid,
                                        'density': round(e.density_ratio, 2),
                                        'speed': round(e.mean_speed_mps, 2)}
                            for e in msg.edges}, time.monotonic())

    def on_route(self, agent, msg):
        if msg.header.frame_id not in ('map', ''):
            return                       # contract §3: routes are in the map frame
        self.live.on_route(agent, [(round(q.pose.position.x, 3), round(q.pose.position.y, 3))
                                   for q in msg.poses], time.monotonic())

    def on_event(self, msg):
        if msg.event == RESET_REQUEST:
            self.live.reset()            # a new world: drop the old picture
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.live.on_event(msg.entity_id, msg.event, msg.reason, stamp, time.monotonic())

    # ------------------------------------------------------------------ API
    def state(self):
        runs = self.runs.snapshot()
        active = runs['active']['id'] if runs['active'] else None
        if active != self._last_active and active is not None:
            self.live.reset()            # a new run starts from an empty view
        self._last_active = active
        return {'live': self.live.snapshot(time.monotonic()), 'runs': runs,
                'commit': os.environ.get('GIT_COMMIT', 'unknown'), 'time': time.time()}

    def api(self, method, path, query, body):
        """Answer one API request; return (status code, JSON-able object)."""
        if method == 'GET' and path == '/api/catalog':
            return 200, {'tasks': catalog.public_catalog(self.root), 'suites': intent.suites()}
        if method == 'GET' and path == '/api/map':
            return 200, static_map()
        if method == 'GET' and path == '/api/state':
            return 200, self.state()
        if method == 'GET' and path == '/api/log':
            return 200, {'log': self.runs.log(query.get('id', [''])[0])}
        if method == 'POST' and path == '/api/plan':
            return 200, intent.parse(body.get('text', ''))
        if method == 'POST' and path == '/api/run':
            items = body.get('plan') or [body]
            runs = [self.runs.submit(i.get('task'), i.get('options'), body.get('source', 'form'))
                    for i in items]
            return 200, {'queued': runs}
        if method == 'POST' and path == '/api/stop':
            return 200, {'stopped': self.runs.stop(body.get('id'))}
        return 404, {'error': 'no such endpoint'}


def make_handler(cc):
    class Handler(BaseHTTPRequestHandler):
        server_version = 'HubControlCenter/1.0'

        def log_message(self, fmt, *args):
            pass                         # keep the terminal for run output

        def _send(self, code, body, ctype='application/json; charset=utf-8'):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('X-Frame-Options', 'DENY')
            self.send_header('Referrer-Policy', 'no-referrer')
            self.send_header('Content-Security-Policy', CSP)
            self.end_headers()
            self.wfile.write(data)

        def _handle(self, method):
            url = urlparse(self.path)
            if method == 'GET' and url.path in STATIC:
                name, ctype = STATIC[url.path]
                with open(os.path.join(WEB_DIR, name), 'rb') as fh:
                    return self._send(200, fh.read(), ctype)
            body = {}
            if method == 'POST':
                # Only this page may start runs: a JSON content type cannot be
                # sent cross-site without a CORS preflight (never answered), and
                # a browser's Origin must be this server.
                origin = self.headers.get('Origin')
                host = self.headers.get('Host', '')
                if not (self.headers.get('Content-Type') or '').startswith('application/json') \
                        or (origin and urlparse(origin).netloc != host):
                    return self._send(403, {'error': 'requests must come from this page'})
                length = int(self.headers.get('Content-Length') or 0)
                if length > MAX_BODY:
                    return self._send(413, {'error': 'request too large'})
                try:
                    body = json.loads(self.rfile.read(length) or b'{}')
                except ValueError:
                    return self._send(400, {'error': 'body must be JSON'})
                if not isinstance(body, dict):
                    return self._send(400, {'error': 'body must be a JSON object'})
            try:
                code, out = cc.api(method, url.path, parse_qs(url.query), body)
            except ValueError as exc:
                code, out = 400, {'error': str(exc)}
            self._send(code, out)

        def do_GET(self):
            self._handle('GET')

        def do_POST(self):
            self._handle('POST')

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = ControlCenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.runs.stop()
        node.server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

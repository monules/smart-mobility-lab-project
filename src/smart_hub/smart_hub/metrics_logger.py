"""metrics_logger (M5): writes per-run CSV/JSON evidence.

Subscribes  /hub/events                 HubEvent           (every node)
            /hub/estop, /hub/safety     Bool, SafetyState  (stop timing, latch state)
            /robot/cmd_vel, /bus/cmd_vel Twist             (final commands)
            /sim/truth                  VehicleStateArray  (simulated actual speed)
Writes      <output_dir>/<run_id>/events.csv and summary.json

The run id comes from scenario_runner's SCENARIO_STARTED / SCENARIO_RESET
events, so a reset starts a new evidence folder. Files are rewritten every
flush_period seconds so a crash still leaves recent evidence. The logger has
no publishers that affect motion.
"""
import os
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from smart_hub.metrics_core import MetricsCore
from smart_hub.ros_common import declare, R10, R100, R1T, reset_ack, RESET_REQUEST
from smart_hub_interfaces.msg import HubEvent, SafetyState, VehicleStateArray
from std_msgs.msg import Bool


class MetricsLogger(Node):
    def __init__(self, **kwargs):
        super().__init__('metrics_logger', **kwargs)
        p = declare(self, {'output_dir': 'output', 'run_id': '', 'scenario_id': '',
                           'seed': 0, 'flush_period': 2.0, 'robot_id': 'robot_1',
                           'bus_id': 'bus_1'})
        self.output_dir = p['output_dir']
        self.scenario_id, self.seed = p['scenario_id'], int(p['seed'])
        self.commit = os.environ.get('GIT_COMMIT', 'unknown')
        self.agent_of = {p['robot_id']: 'robot', p['bus_id']: 'bus'}
        self.metrics = self._new_run(p['run_id'] or 'pending')

        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100)
        # Its only publisher: RESET_ACK events (no influence on motion).
        self.event_pub = self.create_publisher(HubEvent, '/hub/events', R100)
        self.create_subscription(Bool, '/hub/estop', self.on_estop, R1T)
        self.create_subscription(SafetyState, '/hub/safety', self.on_safety, R1T)
        for agent in ('robot', 'bus'):
            self.create_subscription(Twist, '/%s/cmd_vel' % agent,
                                     lambda m, a=agent: self.on_cmd(a, m), R10)
        self.create_subscription(VehicleStateArray, '/sim/truth', self.on_truth, R10)
        self.create_timer(float(p['flush_period']), self.flush)

    def _new_run(self, run_id):
        return MetricsCore(run_id, self.scenario_id, self.seed, self.commit)

    def on_event(self, msg):
        if self.metrics.meta['run_id'] == 'pending' and msg.entity_id == 'scenario_runner' \
                and msg.run_id:
            # Started after the runner's SCENARIO_STARTED: adopt its run id.
            self.metrics.meta['run_id'] = msg.run_id
        if msg.event in ('SCENARIO_STARTED', 'SCENARIO_RESET') and msg.correlation_id:
            if msg.correlation_id != self.metrics.meta['run_id']:
                if self.metrics.events:
                    self.flush()                      # close the previous run
                self.scenario_id = msg.scenario_id or self.scenario_id
                self.metrics = self._new_run(msg.correlation_id)
        self.metrics.on_event(time.monotonic(), msg.entity_id, msg.event,
                              msg.reason, msg.correlation_id)
        if msg.event == RESET_REQUEST:
            self.flush()                     # old run safely on disk first
            self.event_pub.publish(reset_ack(self, msg.correlation_id,
                                             'previous run flushed'))

    def on_estop(self, msg):
        self.metrics.on_estop(time.monotonic(), msg.data)

    def on_safety(self, msg):
        self.metrics.on_safety(time.monotonic(), msg.stop_latched)

    def on_cmd(self, agent, msg):
        self.metrics.on_final_command(time.monotonic(), agent, msg.linear.x, msg.angular.z)

    def on_truth(self, msg):
        now = time.monotonic()
        for v in msg.vehicles:
            agent = self.agent_of.get(v.vehicle_id)
            if agent:
                self.metrics.on_truth(now, agent, v.speed_mps)

    def flush(self):
        if self.metrics.meta['run_id'] == 'pending':
            return                     # keep data in memory until the id is known
        directory = os.path.join(self.output_dir, self.metrics.meta['run_id'])
        try:
            self.metrics.write(directory)
        except OSError as exc:
            self.get_logger().error('cannot write metrics to %s: %s' % (directory, exc))


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

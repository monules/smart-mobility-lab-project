"""mock_candidate_publisher: TEST STAND-IN for M4 robot_manager / M2-A bus_manager.

Publishes a constant candidate velocity on /<agent>/cmd_vel_candidate
(geometry_msgs/TwistStamped, R1) at 20 Hz, stamped with the current time.
It exists only because the real managers are not in this repository yet
(integration rule 3: use mock inputs). Never launch it next to the real
managers: two publishers would fight over the same candidate topic.
"""
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from smart_hub.ros_common import declare, R1


class MockCandidatePublisher(Node):
    def __init__(self, **kwargs):
        super().__init__('mock_candidate_publisher', **kwargs)
        p = declare(self, {'agent': 'robot', 'linear_x': 0.2, 'angular_z': 0.0,
                           'rate_hz': 20.0})
        if p['agent'] not in ('robot', 'bus'):
            raise ValueError('agent must be robot or bus')
        self.vx, self.wz = float(p['linear_x']), float(p['angular_z'])
        self.frame = '%s/base_link' % p['agent']
        self.pub = self.create_publisher(
            TwistStamped, '/%s/cmd_vel_candidate' % p['agent'], R1)
        self.create_timer(1.0 / p['rate_hz'], self.tick)

    def tick(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.twist.linear.x, msg.twist.angular.z = self.vx, self.wz
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockCandidatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

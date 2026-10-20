"""s9_fault_check (M5): runs the S9 recovery test against the live ROS graph.

It plays two roles that do not exist yet in this repository: the robot's
mission manager (it publishes /robot/cmd_vel_candidate) and the operator (it
publishes /hub/estop and calls /safety/clear_stop). It then checks what the
real safety_supervisor, actuator_bridge and scenario_runner do.

Sequence (assignment 16.3, tests T16 and T17):
  1. robot driving at 0.20 m/s; a 0.60 m/s request is clamped to 0.25 m/s
  2. T16: e-stop true -> first zero /robot/cmd_vel within 0.20 s
  3. clear_stop rejected while the e-stop input is still true
  4. e-stop false: motion stays inhibited although the manager still asks
  5. clear_stop rejected while a nonzero command is still requested
  6. clear_stop rejected while robot odometry is stale (injected freeze)
  7. sensors fresh, hazards gone, robot still: clear_stop accepted
  8. after the clear the robot stays stopped (no automatic restart)
  9. a new command moves the robot again
 10. T17 (controller part): candidate stream stops -> zero output
 11. T16 repeated (t16_trials total) for a latency sample set

Results: printed table + <output_dir>/s9_check.json. Exit code 0 only if
every check passed.
"""
import json
import os
import sys
import threading
import time

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from smart_hub.ros_common import declare, R1, R10, R100, R1T
from smart_hub_interfaces.msg import HubEvent, SafetyState, VehicleStateArray
from smart_hub_interfaces.srv import ClearSafetyStop
from std_msgs.msg import Bool, String

T16_LIMIT = 0.20          # s, assignment 3.3 / 21.3
STOPPED = 0.02            # m/s
EPS = 1e-6


class S9Check(Node):
    def __init__(self, **kwargs):
        super().__init__('s9_fault_check', **kwargs)
        p = declare(self, {'output_dir': 'output', 't16_trials': 5,
                           'command_timeout': 0.5, 'robot_id': 'robot_1'})
        self.output_dir, self.trials = p['output_dir'], int(p['t16_trials'])
        self.cmd_timeout, self.robot_id = float(p['command_timeout']), p['robot_id']
        self.lock = threading.Lock()
        self.cand = None                   # (vx, wz) published at 20 Hz, None = silent
        self.estop = False
        self.cmds = []                     # (t, vx, wz) of /robot/cmd_vel
        self.safety = None
        self.truth_speed = None
        self.truth_t = 0.0
        self.sup_latency_ms = []
        self.results = []

        self.cand_pub = self.create_publisher(TwistStamped, '/robot/cmd_vel_candidate', R1)
        self.estop_pub = self.create_publisher(Bool, '/hub/estop', R1T)
        self.inject_pub = self.create_publisher(String, '/scenario/inject', R10)
        self.create_subscription(Twist, '/robot/cmd_vel', self.on_cmd, R10)
        self.create_subscription(SafetyState, '/hub/safety', self.on_safety, R1T)
        self.create_subscription(VehicleStateArray, '/sim/truth', self.on_truth, R10)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100)
        self.clear_cli = self.create_client(ClearSafetyStop, '/safety/clear_stop')
        self.create_timer(0.05, self.publish_inputs)
        self._tick = 0

    # ----------------------------------------------------------- callbacks
    def publish_inputs(self):
        with self.lock:
            cand, estop = self.cand, self.estop
        if cand is not None:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'robot/base_link'
            msg.twist.linear.x, msg.twist.angular.z = cand
            self.cand_pub.publish(msg)
        self._tick += 1
        if self._tick % 2 == 0:
            self.estop_pub.publish(Bool(data=estop))     # 10 Hz heartbeat

    def on_cmd(self, msg):
        with self.lock:
            self.cmds.append((time.monotonic(), msg.linear.x, msg.angular.z))

    def on_safety(self, msg):
        with self.lock:
            self.safety = msg

    def on_truth(self, msg):
        for v in msg.vehicles:
            if v.vehicle_id == self.robot_id:
                with self.lock:
                    self.truth_speed, self.truth_t = v.speed_mps, time.monotonic()

    def on_event(self, msg):
        if msg.event == 'ESTOP_ZERO_OUTPUT' and msg.reason.startswith('receipt_to_zero_ms='):
            with self.lock:
                self.sup_latency_ms.append(float(msg.reason.split('=')[1]))

    # ------------------------------------------------------------- helpers
    def set_cand(self, cand):
        with self.lock:
            self.cand = cand

    def set_estop(self, value):
        with self.lock:
            self.estop = value
        self.estop_pub.publish(Bool(data=value))         # send now, not on next tick

    def last_cmd(self):
        with self.lock:
            return self.cmds[-1] if self.cmds else None

    def cmds_since(self, t0):
        with self.lock:
            return [c for c in self.cmds if c[0] >= t0]

    def wait_for(self, cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cond():
                return True
            time.sleep(0.005)
        return cond()

    def moving_cmd(self, vx):
        c = self.last_cmd()
        return c is not None and abs(c[1] - vx) < 1e-3

    def robot_speed(self):
        with self.lock:
            return self.truth_speed if self.truth_speed is not None else 0.0

    def latched(self):
        with self.lock:
            return bool(self.safety and self.safety.stop_latched)

    def clear(self, who='s9_fault_check'):
        req = ClearSafetyStop.Request(operator_id=who, acknowledgement='S9 test')
        fut = self.clear_cli.call_async(req)
        if not self.wait_for(fut.done, 2.0):
            return None, 'SERVICE_TIMEOUT'
        res = fut.result()
        return res.success, res.reason

    def expect_rejected(self, name, token):
        """Call clear_stop and require a rejection whose reason names token."""
        ok, reason = self.clear()
        return self.check(name, ok is False and token in reason, reason)

    def check(self, name, passed, detail):
        self.results.append({'check': name, 'passed': bool(passed), 'detail': detail})
        print('[%s] %s: %s' % ('PASS' if passed else 'FAIL', name, detail), flush=True)
        return passed

    # ----------------------------------------------------------- sequence
    def drive_until_moving(self, vx=0.2):
        self.set_cand((vx, 0.0))
        return self.wait_for(lambda: self.moving_cmd(vx) and self.robot_speed() > 0.75 * vx, 10.0)

    def estop_trial(self):
        """Assert the e-stop while moving; return (latency s, physical stop s)."""
        t0 = time.monotonic()
        self.set_estop(True)
        got = self.wait_for(lambda: any(abs(c[1]) < EPS and abs(c[2]) < EPS
                                        for c in self.cmds_since(t0)), 2.0)
        zero_t = next(c[0] for c in self.cmds_since(t0)
                      if abs(c[1]) < EPS and abs(c[2]) < EPS) if got else None
        self.wait_for(lambda: self.robot_speed() < STOPPED and self.truth_t > t0, 3.0)
        phys = self.truth_t - t0 if self.robot_speed() < STOPPED else None
        return (zero_t - t0 if zero_t else None), phys

    def stop_and_clear(self):
        self.set_cand((0.0, 0.0))
        self.set_estop(False)
        self.wait_for(lambda: self.robot_speed() < STOPPED, 3.0)
        time.sleep(0.6)                      # nonzero candidate goes stale
        return self.clear()

    def run(self):
        ready = self.wait_for(lambda: self.safety is not None and self.truth_speed is not None
                              and self.clear_cli.service_is_ready(), 20.0)
        if not self.check('system ready', ready, 'safety, truth and clear_stop available'):
            return False
        if self.latched():
            self.check('start unlatched', False,
                       'supervisor already latched: %s' % self.safety.reason)
            return False

        moving = self.drive_until_moving()
        self.check('1a nominal pass-through', moving, 'cmd_vel follows a 0.20 m/s candidate')
        self.set_cand((0.6, 0.0))
        clamped = self.wait_for(lambda: self.moving_cmd(0.25), 2.0)
        self.check('1b robot speed cap', clamped, '0.60 m/s request -> %.3f m/s'
                   % (self.last_cmd()[1] if self.last_cmd() else float('nan')))
        self.drive_until_moving()

        latency, phys = self.estop_trial()
        latencies = [latency]
        self.check('2 T16 stop latency', latency is not None and latency <= T16_LIMIT,
                   'e-stop -> first zero cmd_vel %s s (limit %.2f); sim speed < %.2f after %s s'
                   % (latency and round(latency, 4), T16_LIMIT, STOPPED, phys and round(phys, 3)))

        self.expect_rejected('3 reset rejected: e-stop asserted', 'ESTOP_ASSERTED')

        self.set_estop(False)
        t_false = time.monotonic()
        time.sleep(2.0)                      # manager is still requesting 0.20 m/s
        nonzero = [c for c in self.cmds_since(t_false) if abs(c[1]) > EPS or abs(c[2]) > EPS]
        self.check('4 e-stop false keeps latch', not nonzero and self.latched(),
                   '%d nonzero outputs in 2 s, stop_latched=%s' % (len(nonzero), self.latched()))

        self.expect_rejected('5 reset rejected: command nonzero', 'COMMAND_NONZERO')

        self.set_cand((0.0, 0.0))            # the manager aborts its mission
        self.inject_pub.publish(String(data=json.dumps(
            {'type': 'freeze', 'topic': 'odom', 'duration': 4.0})))
        time.sleep(1.6)                      # odometry now older than 1.0 s
        self.expect_rejected('6 reset rejected: stale odometry', 'ODOM_STALE')

        time.sleep(3.4)                      # freeze over, odometry fresh again
        ok, reason = self.clear()
        unlatched = self.wait_for(lambda: not self.latched(), 0.5)   # next /hub/safety
        self.check('7 reset accepted when safe', ok is True and unlatched,
                   '%s; stop_latched now %s' % (reason, self.latched()))

        t_clear = time.monotonic()
        time.sleep(1.5)
        nonzero = [c for c in self.cmds_since(t_clear) if abs(c[1]) > EPS]
        self.check('8 no automatic restart', not nonzero and self.robot_speed() < STOPPED,
                   '%d nonzero outputs, sim speed %.3f m/s' % (len(nonzero), self.robot_speed()))

        self.check('9 new command moves robot', self.drive_until_moving(),
                   'new 0.20 m/s goal accepted')

        t_lost = time.monotonic()
        self.set_cand(None)                  # controller "crash": stream stops
        got = self.wait_for(lambda: any(abs(c[1]) < EPS for c in self.cmds_since(t_lost)), 2.0)
        zero_t = next((c[0] for c in self.cmds_since(t_lost) if abs(c[1]) < EPS), None)
        limit = self.cmd_timeout + 0.25
        self.check('10 T17 controller loss -> zero', got and zero_t - t_lost <= limit,
                   'candidate stream stopped -> zero after %s s '
                   '(timeout %.2f + tick/transport margin)'
                   % (zero_t and round(zero_t - t_lost, 3), self.cmd_timeout))

        for _ in range(self.trials - 1):
            self.stop_and_clear()
            if not self.drive_until_moving():
                latencies.append(None)
                continue
            latencies.append(self.estop_trial()[0])
        self.stop_and_clear()
        good = [x for x in latencies if x is not None]
        self.check('11 T16 repeated', len(good) == len(latencies) and max(good) <= T16_LIMIT,
                   'n=%d max=%.4f s mean=%.4f s; supervisor receipt->zero max %.3f ms'
                   % (len(good), max(good) if good else -1, sum(good) / max(1, len(good)),
                      max(self.sup_latency_ms) if self.sup_latency_ms else -1))
        self.t16 = {'external_latency_s': latencies,
                    'supervisor_receipt_to_zero_ms': self.sup_latency_ms}
        return all(r['passed'] for r in self.results)

    def write(self, ok):
        os.makedirs(self.output_dir, exist_ok=True)
        path = os.path.join(self.output_dir, 's9_check.json')
        with open(path, 'w') as fh:
            json.dump({'passed': ok, 'checks': self.results,
                       't16': getattr(self, 't16', None),
                       'commit': os.environ.get('GIT_COMMIT', 'unknown')}, fh, indent=2)
        print('S9 %s; evidence: %s' % ('PASSED' if ok else 'FAILED', path), flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = S9Check()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    ok = False
    try:
        ok = node.run()
    finally:
        node.set_cand(None)
        node.write(ok)
        # Stop the spin thread and wait for it before tearing down; exiting
        # while it is inside rclpy aborts the process (exit code -6).
        executor.shutdown()
        spinner.join(timeout=5.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

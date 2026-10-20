"""Smoke tests of parking_manager.py / traffic_monitor.py against rclpy STUBS.

Skipped automatically when a real rclpy is importable (use colcon test / the
launch_testing integration tests there instead).
"""
import importlib.util
import json
import time
import unittest

import ros_stubs

REAL_ROS = importlib.util.find_spec('rclpy') is not None and \
    'rclpy.node' not in __import__('sys').modules


@unittest.skipIf(REAL_ROS, 'real rclpy present: stub smoke test not applicable')
class NodeSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        ros_stubs.install()
        from smart_hub.parking_manager import ParkingManager
        from smart_hub.traffic_monitor import TrafficMonitor
        cls.ParkingManager, cls.TrafficMonitor = ParkingManager, TrafficMonitor

    # ---------- helpers ----------
    @staticmethod
    def stamp_ns(node, offset_s=0.0):
        return node.get_clock().now().nanoseconds + int(offset_s * 1e9)

    def obs_msg(self, node, states, offset_s=0.0):
        m = ros_stubs.SlotObservationArray()
        ns = self.stamp_ns(node, offset_s)
        m.header.stamp.sec, m.header.stamp.nanosec = ns // 10**9, ns % 10**9
        m.slots = [ros_stubs.SlotObservation(slot_id=k, occupied=(v == 'OCCUPIED'))
                   for k, v in states.items()]
        return m

    def state_of(self, node):
        return {s.slot_id: s.state for s in node.pubs['/parking/state'].last.slots}

    # ---------- parking ----------
    def test_parking_node_wiring_and_flow(self):
        n = self.ParkingManager(parameter_overrides={'debounce_sec': 0.0})
        self.assertIn('/parking/reserve_slot', n.srvs)
        self.assertIn('/parking/release_slot', n.srvs)
        self.assertIn('/parking/observed', n.subs)
        for topic in ('/parking/state', '/parking/barrier_request', '/hub/events'):
            self.assertIn(topic, n.pubs)
        self.assertEqual(n.pubs['/parking/state'].qos.durability, 'TRANSIENT_LOCAL')
        self.assertEqual(n.pubs['/parking/state'].qos.depth, 1)
        self.assertEqual(n.subs['/parking/observed'][2].depth, 10)
        self.assertEqual(set(self.state_of(n).values()), {'UNKNOWN'})
        self.assertFalse(n.pubs['/parking/barrier_request'].last.data)

        n.on_observed(self.obs_msg(n, {'P1': 'OCCUPIED', 'P2': 'FREE', 'P3': 'FREE'}))
        self.assertEqual(self.state_of(n), {'P1': 'OCCUPIED', 'P2': 'FREE', 'P3': 'FREE'})
        self.assertEqual(n.pubs['/parking/state'].last.available_count, 2)

        Req, Resp = ros_stubs.ReserveParking.Request, ros_stubs.ReserveParking.Response
        r1 = n.on_reserve(Req(request_id='a', vehicle_id='C1', preferred_slot='P2',
                              ttl_sec=30.0), Resp())
        r2 = n.on_reserve(Req(request_id='b', vehicle_id='C2', preferred_slot='P2',
                              ttl_sec=30.0), Resp())
        self.assertTrue(r1.success and r2.success)
        self.assertEqual(sorted([r1.slot_id, r2.slot_id]), ['P2', 'P3'])
        exp = r1.expires_at.sec + r1.expires_at.nanosec * 1e-9
        self.assertAlmostEqual(exp - time.time(), 30.0, delta=1.0)
        self.assertEqual(n.pubs['/parking/state'].last.available_count, 0)
        dup = n.on_reserve(Req(request_id='a', vehicle_id='C1', preferred_slot='P2',
                               ttl_sec=30.0), Resp())
        self.assertEqual(dup.reservation_id, r1.reservation_id)
        bad_ttl = n.on_reserve(Req(request_id='c', vehicle_id='C3', preferred_slot='',
                                   ttl_sec=3.0), Resp())
        self.assertEqual((bad_ttl.success, bad_ttl.reason), (False, 'INVALID_TTL'))

        events = [(e.event, e.correlation_id) for e in n.pubs['/hub/events'].sent]
        self.assertIn(('RESERVED', r1.reservation_id), events)
        self.assertTrue(any(e == 'RESERVE_REPLAY' for e, _ in events))

        RReq, RResp = ros_stubs.ReleaseParking.Request, ros_stubs.ReleaseParking.Response
        wrong = n.on_release(RReq(request_id='x', reservation_id='nope', vehicle_id='C1'),
                             RResp())
        self.assertEqual((wrong.success, wrong.reason), (False, 'UNKNOWN_TOKEN'))
        ok = n.on_release(RReq(request_id='y', reservation_id=r1.reservation_id,
                               vehicle_id='C1'), RResp())
        self.assertTrue(ok.success)
        self.assertEqual(self.state_of(n)[r1.slot_id], 'FREE')

    def test_parking_drops_old_and_future_stamped_observations(self):
        n = self.ParkingManager(parameter_overrides={'debounce_sec': 0.0})
        n.on_observed(self.obs_msg(n, {'P2': 'FREE'}, offset_s=-5.0))     # 5 s old
        n.on_observed(self.obs_msg(n, {'P2': 'FREE'}, offset_s=+5.0))     # from the future
        self.assertEqual(self.state_of(n)['P2'], 'UNKNOWN')
        n.on_observed(self.obs_msg(n, {'P2': 'FREE'}, offset_s=-0.2))
        self.assertEqual(self.state_of(n)['P2'], 'FREE')

    def test_parking_barrier_path(self):
        n = self.ParkingManager(parameter_overrides={'debounce_sec': 0.0})
        n.on_observed(self.obs_msg(n, {'P2': 'FREE', 'P3': 'FREE', 'P1': 'FREE'}))
        Req, Resp = ros_stubs.ReserveParking.Request, ros_stubs.ReserveParking.Response
        r = n.on_reserve(Req(request_id='a', vehicle_id='C1', preferred_slot='P2',
                             ttl_sec=60.0), Resp())
        claim = lambda v, t: n.on_claim(ros_stubs.String(data='%s|%s' % (v, t)))  # noqa
        claim('C1', r.reservation_id)                                     # no entrance sensor yet
        self.assertFalse(n.pubs['/parking/barrier_request'].last.data)
        n.on_entrance(ros_stubs.Bool(data=True))
        claim('C2', r.reservation_id)                                     # wrong vehicle
        self.assertFalse(n.pubs['/parking/barrier_request'].last.data)
        claim('C1', r.reservation_id)
        self.assertTrue(n.pubs['/parking/barrier_request'].last.data)
        reasons = [e.reason for e in n.pubs['/hub/events'].sent if e.event == 'BARRIER_DENIED']
        self.assertEqual(reasons, ['ENTRANCE_SENSOR_STALE', 'VEHICLE_MISMATCH'])

    def test_parking_invalid_parameters_rejected_at_startup(self):
        for bad in ({'reservation_ttl_sec': 500.0}, {'reservation_ttl_sec': 1.0},
                    {'obs_freshness_sec': 0.0}, {'state_publish_hz': 0.0},
                    {'debounce_sec': -1.0}, {'slot_ids': ['P1', 'P1']}):
            with self.assertRaises(ValueError, msg=str(bad)):
                self.ParkingManager(parameter_overrides=bad)

    def test_parking_tick_expires_reservations_and_republishes(self):
        n = self.ParkingManager(parameter_overrides={'debounce_sec': 0.0})
        n.on_observed(self.obs_msg(n, {'P2': 'FREE'}))
        Req, Resp = ros_stubs.ReserveParking.Request, ros_stubs.ReserveParking.Response
        r = n.on_reserve(Req(request_id='a', vehicle_id='C1', preferred_slot='P2',
                             ttl_sec=30.0), Resp())
        self.assertEqual(self.state_of(n)['P2'], 'RESERVED')
        before = len(n.pubs['/parking/state'].sent)
        n.on_tick()
        self.assertGreaterEqual(len(n.pubs['/parking/state'].sent), before)
        n.engine._slots['P2'].expiry = time.monotonic() - 1.0            # force TTL elapsed
        n.engine._slots['P2'].last_seen = time.monotonic()               # sensor still fresh
        n.on_tick()
        self.assertEqual(self.state_of(n)['P2'], 'FREE')
        self.assertIn('RESERVATION_EXPIRED',
                      [e.event for e in n.pubs['/hub/events'].sent])

    # ---------- traffic ----------
    def veh_msg(self, node, vehicles, i=0):
        m = ros_stubs.VehicleStateArray()
        ns = self.stamp_ns(node, -0.5 + 0.05 * i)
        m.header.stamp.sec, m.header.stamp.nanosec = ns // 10**9, ns % 10**9
        for vid, kind, x, y, sp, edge in vehicles:
            v = ros_stubs.VehicleState(vehicle_id=vid, kind=kind, speed_mps=sp,
                                       edge_id=edge, valid=True)
            v.pose.x, v.pose.y = x, y
            m.vehicles.append(v)
        return m

    def test_traffic_node_publishes_edge_states_labels_and_events(self):
        n = self.TrafficMonitor()
        self.assertIn('/hub/vehicles', n.subs)
        self.assertEqual(n.subs['/hub/vehicles'][2].depth, 10)
        cars = [('c%d' % i, 'car', 5.5 + 0.8 * i, 2.0, 0.1, 'Cs-J') for i in range(4)]
        for i in range(6):
            n.on_vehicles(self.veh_msg(n, cars, i))
        n.publish_now()
        out = n.pubs['/traffic/edges'].last
        e = {x.edge_id: x for x in out.edges}['Cs-J']
        self.assertEqual((e.valid, e.blocked, e.density_ratio), (True, False, 1.0))
        self.assertAlmostEqual(e.mean_speed_mps, 0.1, places=3)
        self.assertEqual(len(out.edges), 8)
        labels = json.loads(n.pubs['/traffic/edge_labels'].last.data)['edges']
        self.assertEqual(labels['Cs-J']['label'], 'CONGESTED')
        self.assertEqual(labels['A-Cs']['label'], 'NORMAL')
        ev = [(x.entity_id, x.event) for x in n.pubs['/hub/events'].sent]
        self.assertIn(('Cs-J', 'CONGESTED'), ev)
        # obstacle O in the same stream -> BLOCKED on the next publication
        n.on_vehicles(self.veh_msg(n, cars + [('O', 'obstacle', 7.0, 2.0, 0.0, '')], 7))
        n.publish_now()
        e = {x.edge_id: x for x in n.pubs['/traffic/edges'].last.edges}['Cs-J']
        self.assertTrue(e.blocked)
        self.assertEqual(json.loads(n.pubs['/traffic/edge_labels'].last.data)
                         ['edges']['Cs-J']['label'], 'BLOCKED')

    def test_traffic_watchdog_publishes_unknown_when_feed_stops(self):
        n = self.TrafficMonitor()
        for i in range(6):
            n.on_vehicles(self.veh_msg(n, [], i))
        n.publish_now()
        self.assertTrue(all(x.valid for x in n.pubs['/traffic/edges'].last.edges))
        count = len(n.pubs['/traffic/edges'].sent)
        n.on_watchdog()
        self.assertEqual(len(n.pubs['/traffic/edges'].sent), count)       # still fresh
        time.sleep(1.15)
        n.on_watchdog()                                                   # stale -> publish now
        out = n.pubs['/traffic/edges'].last
        self.assertEqual(len(n.pubs['/traffic/edges'].sent), count + 1)
        for x in out.edges:
            self.assertEqual((x.valid, x.mean_speed_mps, x.density_ratio), (False, -1.0, -1.0))
        n.on_watchdog()                                                   # no repeat flood
        self.assertEqual(len(n.pubs['/traffic/edges'].sent), count + 1)

    def test_traffic_header_carries_source_time_not_publication_time(self):
        n = self.TrafficMonitor()
        for i in range(6):
            n.on_vehicles(self.veh_msg(n, [], i))
        stamp = n.engine.last_observation_stamp()
        time.sleep(0.3)
        n.publish_now()
        h = n.pubs['/traffic/edges'].last.header.stamp
        self.assertAlmostEqual(h.sec + h.nanosec * 1e-9, stamp, places=3)

    def test_traffic_invalid_parameters_rejected_at_startup(self):
        for bad in ({'stale_sec': 3.0}, {'publish_hz': 0.0},
                    {'congestion_density_threshold': 0.0}, {'slow_fraction': 2.0}):
            with self.assertRaises(ValueError, msg=str(bad)):
                self.TrafficMonitor(parameter_overrides=bad)


if __name__ == '__main__':
    unittest.main()

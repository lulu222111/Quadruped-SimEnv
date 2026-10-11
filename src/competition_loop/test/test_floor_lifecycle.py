import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from floor_lifecycle import FloorLifecycle


class FloorLifecycleTest(unittest.TestCase):
    def setUp(self):
        self.scene = {'door_ids': [{'kind': 'main_entrance', 'floor_index': 0}],
                      'elevators': [{'id': 'lift', 'served_floors': [0, 1, 2]}]}
        self.core = FloorLifecycle(self.scene)
        self.token = self.core.activate(0, 100.)

    def proof(self, token=None, stamp=101., confirmed=True):
        self.core.observe_status(token or self.token, stamp, True, confirmed, 101.)

    def arm(self):
        self.core.observe_bool(False, 100.1)
        self.core.observe_bool(True, 101.)

    def test_current_floor_is_not_building_complete(self):
        self.arm()
        self.proof()
        self.assertTrue(self.core.consume(101.1))
        self.assertEqual(self.core.remaining, (1, 2))
        self.assertEqual(self.core.completed, {0})
        self.assertFalse(self.core.consume(101.2))

    def test_bool_alone_never_completes(self):
        self.arm()
        self.assertFalse(self.core.consume(101.))

    def test_latched_true_without_reset_rejected(self):
        self.core.observe_bool(True, 101.)
        self.proof()
        self.assertFalse(self.core.consume(101.))

    def test_transient_no_frontier_without_planner_debounce_rejected(self):
        self.arm()
        self.proof(confirmed=False)
        self.assertFalse(self.core.consume(101.))

    def test_old_floor_and_restart_evidence_rejected(self):
        old = self.token
        self.token = self.core.activate(1, 100.5)
        self.arm()
        self.proof(token=old)
        self.assertFalse(self.core.consume(101.))
        restarted = FloorLifecycle(self.scene)
        self.assertNotEqual(restarted.activate(1, 100.5), self.token)

    def test_stale_future_zero_nan_evidence_rejected(self):
        self.arm()
        for stamp in [99., 102., 0., float('nan'), float('inf')]:
            self.proof(stamp=stamp)
            self.assertFalse(self.core.consume(101.))

    def test_completion_expires(self):
        self.arm()
        self.proof()
        self.assertFalse(self.core.consume(103.))

    def test_withdrawal_wins_over_delayed_true(self):
        self.arm()
        self.proof()
        self.core.observe_status(self.token, 101.1, False, False, 101.1)
        self.core.observe_bool(True, 101.2)
        self.assertFalse(self.core.consume(101.2))

    def test_old_status_does_not_overwrite_new_evidence(self):
        self.arm()
        self.proof()
        self.core.observe_status(self.token, 100.5, False, False, 101.)
        self.assertTrue(self.core.consume(101.))

    def test_floor_reset_clears_pending_completion(self):
        self.arm()
        self.proof()
        self.core.activate(1, 101.1)
        self.assertFalse(self.core.consume(101.2))

    def test_bool_and_status_arrival_order_independent(self):
        self.proof()
        self.arm()
        self.assertTrue(self.core.consume(101.))

    def test_invalid_public_semantics_rejected(self):
        self.scene['elevators'][0]['served_floors'] = [True]
        with self.assertRaises(ValueError):
            FloorLifecycle(self.scene)


if __name__ == '__main__':
    unittest.main()

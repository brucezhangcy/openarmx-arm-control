import unittest

from safety import (
    TargetRejected,
    check_target,
    max_joint_change,
    parse_command,
    resolve_target,
    validate_safe_box,
)

BOX = {"x": [-0.2, 0.2], "y": [-0.2, 0.2], "z": [-0.1, 0.3]}
HOME = (0.3, -0.2, 0.4)


class ResolveTarget(unittest.TestCase):
    def test_absolute_passes_through(self):
        self.assertEqual(resolve_target([0.1, 0.2, 0.3], (9, 9, 9), False), (0.1, 0.2, 0.3))

    def test_relative_adds_to_current(self):
        got = resolve_target([0.0, 0.0, 0.03], (0.3, -0.2, 0.4), True)
        self.assertAlmostEqual(got[2], 0.43)

    def test_wrong_count(self):
        with self.assertRaises(ValueError):
            resolve_target([0.1, 0.2], HOME, False)

    def test_nan_rejected(self):
        with self.assertRaises(ValueError):
            resolve_target([float("nan"), 0, 0], HOME, False)


class CheckTarget(unittest.TestCase):
    def test_small_move_inside_box_ok(self):
        check_target((0.32, -0.2, 0.45), HOME, HOME, BOX, 0.15)

    def test_outside_box_rejected(self):
        with self.assertRaises(TargetRejected):
            check_target((0.3, -0.2, 0.25), HOME, HOME, BOX, 1.0)  # 0.15 below home

    def test_typo_scale_rejected(self):
        with self.assertRaises(TargetRejected):
            check_target((1.0, -0.2, 0.4), HOME, HOME, BOX, 1.0)

    def test_step_limit(self):
        # Inside the box, but 0.2 m from the current position.
        with self.assertRaises(TargetRejected):
            check_target((0.3, -0.2, 0.6), HOME, (0.3, -0.2, 0.4), BOX, 0.15)

    def test_box_edge_inclusive(self):
        check_target((0.5, -0.2, 0.4), HOME, (0.45, -0.2, 0.4), BOX, 0.15)


class MaxJointChange(unittest.TestCase):
    def test_largest_absolute_change(self):
        # The joint 5 swing seen in rung 1: -0.98 rad from home.
        start = [0.0] * 7
        end = [-0.11, -0.09, 0.82, 0.54, -0.98, 0.68, -0.35]
        self.assertAlmostEqual(max_joint_change(start, end), 0.98)

    def test_length_mismatch(self):
        with self.assertRaises(ValueError):
            max_joint_change([0.0] * 7, [0.0] * 6)


class ValidateBox(unittest.TestCase):
    def test_box_must_contain_home(self):
        with self.assertRaises(ValueError):
            validate_safe_box({"x": [0.05, 0.2], "y": [-0.2, 0.2], "z": [-0.1, 0.3]})


class ParseCommand(unittest.TestCase):
    def test_absolute(self):
        self.assertEqual(parse_command("0.3 -0.2 0.5"), ("move", [0.3, -0.2, 0.5]))

    def test_rel(self):
        self.assertEqual(parse_command("rel 0 0 0.03"), ("rel", [0.0, 0.0, 0.03]))

    def test_words(self):
        self.assertEqual(parse_command("POS"), ("pos", []))
        self.assertEqual(parse_command("q"), ("quit", []))
        self.assertEqual(parse_command("   "), ("empty", []))

    def test_garbage(self):
        with self.assertRaises(ValueError):
            parse_command("go there")


if __name__ == "__main__":
    unittest.main()

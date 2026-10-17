"""Target checks and command parsing for ee_goto.py.

Kept free of ROS imports so it can be unit-tested on any machine
(python3 -m unittest test_safety).
"""

import math


class TargetRejected(ValueError):
    """The target failed a safety check; nothing should be sent to MoveIt."""


def resolve_target(values, current, relative):
    """Turn typed numbers into an absolute (x, y, z) in the planning frame."""
    if len(values) != 3:
        raise ValueError("need exactly three numbers: x y z")
    if not all(math.isfinite(v) for v in values):
        raise ValueError("coordinates must be finite numbers")
    if relative:
        return tuple(c + v for c, v in zip(current, values))
    return tuple(values)


def check_target(target, home, current, safe_box, max_step):
    """Raise TargetRejected unless the target is inside the safe box and within one step.

    safe_box holds, per axis, the allowed [low, high] offset in metres from the
    home gripper position. max_step is the largest straight-line distance one
    move may cover from the current gripper position.
    """
    for i, axis in enumerate("xyz"):
        low, high = safe_box[axis]
        offset = target[i] - home[i]
        if not low <= offset <= high:
            raise TargetRejected(
                f"{axis}={target[i]:.3f} m is {offset:+.3f} m from home; "
                f"the safe box allows {low:+.3f} to {high:+.3f}"
            )
    step = math.dist(target, current)
    if step > max_step:
        raise TargetRejected(
            f"this move covers {step:.3f} m; the limit is {max_step:.3f} m per move, "
            "so split it into smaller steps"
        )


def max_joint_change(start, end):
    """Largest absolute change of any joint between two joint vectors (rad)."""
    if len(start) != len(end):
        raise ValueError("joint vectors differ in length")
    return max((abs(b - a) for a, b in zip(start, end)), default=0.0)


def validate_safe_box(safe_box):
    """Fail fast on a malformed safe_box section in config.yaml."""
    for axis in "xyz":
        low, high = safe_box[axis]
        if not low <= 0.0 <= high:
            raise ValueError(f"safe_box.{axis} must contain 0 (home itself): got [{low}, {high}]")


def parse_command(line):
    """Split one typed line into (command, numbers).

    Commands: 'move' (three numbers), 'rel' (three numbers), 'pos', 'home',
    'help', 'quit'. Returns ('empty', []) for a blank line.
    """
    words = line.strip().split()
    if not words:
        return "empty", []
    head = words[0].lower()
    if head in ("q", "quit", "exit"):
        return "quit", []
    if head in ("pos", "home", "help"):
        if len(words) != 1:
            raise ValueError(f"'{head}' takes no arguments")
        return head, []
    if head == "rel":
        return "rel", [float(w) for w in words[1:]]
    return "move", [float(w) for w in words]

#!/usr/bin/env python3
"""Move the OpenArmX right gripper to a typed x, y, z position through MoveIt.

Runs on the arm PC with the MoveIt stack already up, for example:

    ros2 launch openarmx_bimanual_moveit_config demo.launch.py can_fd:=false

Every motion is planned first and shown; it is executed only after you type y.
See README.md for the session routine and the test ladder.
"""

import argparse
import sys
from pathlib import Path

import rclpy
import tf2_ros
import yaml
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPositionFK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive

from safety import (
    TargetRejected,
    check_target,
    max_joint_change,
    parse_command,
    resolve_target,
    validate_safe_box,
)

HERE = Path(__file__).resolve().parent

# A plan whose joints move less than this is accepted without trying more candidates.
GOOD_ENOUGH_SWING_RAD = 0.15

ERROR_NAMES = {
    value: name
    for name, value in vars(MoveItErrorCodes).items()
    if name.isupper() and isinstance(value, int)
}

HELP = """commands:
  x y z          go to an absolute position (metres, in the planning frame)
  rel dx dy dz   move relative to the current gripper position
  pos            print the current gripper position
  home           move the right arm back to all-zero joints
  help           show this list
  quit           leave (the arm keeps holding its pose; see README to shut down)"""


class PlanningFailed(RuntimeError):
    pass


def fmt(xyz):
    return "x={:+.3f} y={:+.3f} z={:+.3f}".format(*xyz)


class ArmClient(Node):
    """Thin wrapper around the MoveIt interfaces that move_group already provides."""

    def __init__(self, cfg):
        super().__init__("ee_goto")
        self.cfg = cfg
        self.frame = cfg["planning_frame"]
        self.link = cfg["tip_link"]
        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.exec_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    # -- plumbing -----------------------------------------------------------

    def _wait(self, future, timeout, what):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError(f"no answer from {what} within {timeout:.0f} s")
        return future.result()

    def _spin_for(self, seconds):
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_ready(self, timeout=15.0):
        checks = [
            (self.move_client.wait_for_server, "/move_action"),
            (self.exec_client.wait_for_server, "/execute_trajectory"),
            (self.fk_client.wait_for_service, "/compute_fk"),
            (self.scene_client.wait_for_service, "/apply_planning_scene"),
        ]
        for wait, name in checks:
            if not wait(timeout_sec=timeout):
                raise TimeoutError(f"{name} not available: is the MoveIt stack running?")

    # -- reading the arm ----------------------------------------------------

    def current_position(self, timeout=3.0):
        """Gripper position from TF, i.e. from the measured joint angles."""
        end = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while not self.tf_buffer.can_transform(self.frame, self.link, Time()):
            if self.get_clock().now().nanoseconds > end:
                raise TimeoutError(f"no transform {self.frame} -> {self.link} (is joint_states publishing?)")
            rclpy.spin_once(self, timeout_sec=0.05)
        t = self.tf_buffer.lookup_transform(self.frame, self.link, Time()).transform.translation
        return (t.x, t.y, t.z)

    def home_pose(self):
        """Gripper pose with all right-arm joints at zero, from MoveIt's forward kinematics."""
        req = GetPositionFK.Request()
        req.header.frame_id = self.frame
        req.fk_link_names = [self.link]
        req.robot_state.joint_state.name = list(self.cfg["joints"])
        req.robot_state.joint_state.position = [0.0] * len(self.cfg["joints"])
        res = self._wait(self.fk_client.call_async(req), 5.0, "/compute_fk")
        if res.error_code.val != MoveItErrorCodes.SUCCESS or not res.pose_stamped:
            raise PlanningFailed(f"forward kinematics failed: {ERROR_NAMES.get(res.error_code.val, res.error_code.val)}")
        pose = res.pose_stamped[0].pose
        return (pose.position.x, pose.position.y, pose.position.z), pose.orientation

    # -- planning scene -----------------------------------------------------

    def add_table(self):
        t = self.cfg["table"]
        top = t["top_z_m"] + t["margin_m"]
        box = SolidPrimitive(
            type=SolidPrimitive.BOX,
            dimensions=[t["size_xy_m"][0], t["size_xy_m"][1], t["thickness_m"]],
        )
        obj = CollisionObject(id="table", operation=CollisionObject.ADD)
        obj.header.frame_id = self.frame
        obj.primitives = [box]
        obj.primitive_poses = [Pose(
            position=Point(x=t["center_xy_m"][0], y=t["center_xy_m"][1], z=top - t["thickness_m"] / 2),
            orientation=Quaternion(w=1.0),
        )]
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [obj]
        res = self._wait(self.scene_client.call_async(ApplyPlanningScene.Request(scene=scene)), 5.0, "/apply_planning_scene")
        if not res.success:
            raise RuntimeError("MoveIt refused the table collision object")
        return top

    # -- planning and execution ---------------------------------------------

    def _plan(self, constraints):
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = self.cfg["group"]
        r.num_planning_attempts = int(self.cfg["planning_attempts"])
        r.allowed_planning_time = float(self.cfg["planning_time_s"])
        r.max_velocity_scaling_factor = float(self.cfg["velocity_scaling"])
        r.max_acceleration_scaling_factor = float(self.cfg["acceleration_scaling"])
        r.start_state.is_diff = True  # plan from the current measured state
        r.goal_constraints = [constraints]
        r.workspace_parameters.header.frame_id = self.frame
        r.workspace_parameters.min_corner.x = r.workspace_parameters.min_corner.y = -2.0
        r.workspace_parameters.min_corner.z = -1.0
        r.workspace_parameters.max_corner.x = r.workspace_parameters.max_corner.y = 2.0
        r.workspace_parameters.max_corner.z = 3.0
        goal.planning_options.plan_only = True  # execution happens separately, after confirmation

        handle = self._wait(self.move_client.send_goal_async(goal), 10.0, "/move_action")
        if not handle.accepted:
            raise PlanningFailed("move_group rejected the planning request")
        timeout = float(self.cfg["planning_time_s"]) * int(self.cfg["planning_attempts"]) + 10.0
        result = self._wait(handle.get_result_async(), timeout, "/move_action").result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise PlanningFailed(ERROR_NAMES.get(result.error_code.val, str(result.error_code.val)))
        return result.planned_trajectory

    def plan_to_position(self, target, orientation):
        """Plan the gripper to target; orientation=None means position only."""
        c = Constraints()
        pc = PositionConstraint(link_name=self.link, weight=1.0)
        pc.header.frame_id = self.frame
        pc.constraint_region.primitives = [SolidPrimitive(
            type=SolidPrimitive.SPHERE, dimensions=[float(self.cfg["position_tolerance_m"])])]
        pc.constraint_region.primitive_poses = [Pose(
            position=Point(x=target[0], y=target[1], z=target[2]), orientation=Quaternion(w=1.0))]
        c.position_constraints = [pc]
        if orientation is not None:
            tol = float(self.cfg["orientation_tolerance_rad"])
            oc = OrientationConstraint(
                link_name=self.link, orientation=orientation, weight=1.0,
                absolute_x_axis_tolerance=tol, absolute_y_axis_tolerance=tol, absolute_z_axis_tolerance=tol,
            )
            oc.header.frame_id = self.frame
            c.orientation_constraints = [oc]
        return self._plan(c)

    def plan_home(self):
        c = Constraints()
        c.joint_constraints = [
            JointConstraint(joint_name=j, position=0.0, tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            for j in self.cfg["joints"]
        ]
        return self._plan(c)

    def execute(self, trajectory):
        """Run exactly the trajectory that was shown. Ctrl+C asks MoveIt to stop it."""
        handle = self._wait(self.exec_client.send_goal_async(ExecuteTrajectory.Goal(trajectory=trajectory)),
                            10.0, "/execute_trajectory")
        if not handle.accepted:
            raise RuntimeError("MoveIt rejected the trajectory")
        future = handle.get_result_async()
        try:
            while not future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            print("\nStopping: asking MoveIt to cancel the motion...")
            self._wait(handle.cancel_goal_async(), 5.0, "cancel")
            raise
        code = future.result().result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"execution ended with {ERROR_NAMES.get(code, code)}")


def joint_swing(trajectory):
    """Largest distance any joint gets from its start value, anywhere along the path."""
    points = trajectory.joint_trajectory.points
    if not points:
        return 0.0
    start = points[0].positions
    return max(max_joint_change(start, p.positions) for p in points)


def summarize(trajectory):
    jt = trajectory.joint_trajectory
    if not jt.points:
        return "empty trajectory (already there?)"
    last = jt.points[-1]
    duration = last.time_from_start.sec + last.time_from_start.nanosec * 1e-9
    joints = ", ".join(f"{n.split('_')[-1]}={p:+.2f}" for n, p in zip(jt.joint_names, last.positions))
    return f"{duration:.1f} s, {len(jt.points)} points, final joints [{joints}] rad"


def confirm(prompt="Execute? [y/N] "):
    return input(prompt).strip().lower() == "y"


class Session:
    """The interactive loop; move_to() is the one reusable entry point."""

    def __init__(self, arm, cfg, plan_only):
        self.arm = arm
        self.cfg = cfg
        self.plan_only = plan_only
        self.home_xyz, self.home_quat = arm.home_pose()
        self.table_on = False

    def move_to(self, x, y, z, relative=False, execute=False, ask=None):
        """Plan the gripper to (x, y, z); if execute, run that same plan.

        ask, if given, is called after the plan is printed and must return True
        for the motion to run. Returns (target, reached); reached is None when
        nothing moved.
        """
        current = self.arm.current_position()
        target = resolve_target([x, y, z], current, relative)
        check_target(target, self.home_xyz, current, self.cfg["safe_box"], float(self.cfg["max_step_m"]))

        try:
            traj, swing = self._best_plan(target, self.home_quat)
            mode = "home orientation"
        except PlanningFailed as e:
            print(f"  no plan keeping the home orientation ({e}); trying position only...")
            traj, swing = self._best_plan(target, None)
            mode = "POSITION ONLY: the gripper may turn"
        limit = float(self.cfg["max_joint_change_rad"])
        print(f"  target  {fmt(target)}")
        print(f"  plan    {summarize(traj)}  [{mode}]")
        print(f"  largest joint motion {swing:.2f} rad (limit {limit:.2f})")
        if swing > limit:
            raise TargetRejected(
                f"even the best of {self.cfg['plan_candidates']} plans moves a joint {swing:.2f} rad; "
                "try a smaller step or a different target"
            )

        if not execute:
            return target, None
        if ask is not None and not ask():
            print("  not executed")
            return target, None
        self.arm.execute(traj)
        self.arm._spin_for(float(self.cfg["settle_s"]))
        reached = self.arm.current_position()
        error = sum((a - b) ** 2 for a, b in zip(reached, target)) ** 0.5
        print(f"  reached {fmt(reached)}  error {error * 1000:.0f} mm")
        return target, reached

    def _best_plan(self, target, orientation):
        """Plan several times and keep the plan with the smallest joint motion.

        The arm has 7 joints, so MoveIt can reach one gripper position with many
        joint configurations and picks one at random; without this, a 3 cm nudge
        can swing the elbow and wrist by a radian.
        """
        best, last_error = None, None
        for _ in range(int(self.cfg["plan_candidates"])):
            try:
                traj = self.arm.plan_to_position(target, orientation)
            except PlanningFailed as e:
                last_error = e
                continue
            swing = joint_swing(traj)
            if best is None or swing < best[1]:
                best = (traj, swing)
            if swing <= GOOD_ENOUGH_SWING_RAD:
                break
        if best is None:
            raise last_error
        return best

    def go_home(self):
        traj = self.arm.plan_home()
        print(f"  plan    {summarize(traj)}  [all-zero joints]")
        if self.plan_only or not confirm():
            print("  not executed")
            return
        self.arm.execute(traj)
        self.arm._spin_for(float(self.cfg["settle_s"]))
        print(f"  gripper now at {fmt(self.arm.current_position())}")

    def run(self):
        print(f"frame {self.arm.frame}, gripper link {self.arm.link}")
        print(f"home gripper position     {fmt(self.home_xyz)}")
        print(f"current gripper position  {fmt(self.arm.current_position())}")
        if self.cfg["table"]["enabled"]:
            top = self.arm.add_table()
            self.table_on = True
            print(f"table added to the planning scene, top at z={top:+.3f}")
        else:
            print("WARNING: table not in the planning scene (table.enabled is false in config.yaml)")
        if self.plan_only:
            print("PLAN-ONLY mode: nothing will move.")
        print(HELP)

        while True:
            try:
                line = input("target> ")
            except (EOFError, KeyboardInterrupt):
                print()
                return
            try:
                cmd, nums = parse_command(line)
                if cmd == "empty":
                    continue
                if cmd == "quit":
                    return
                if cmd == "help":
                    print(HELP)
                elif cmd == "pos":
                    print(f"  {fmt(self.arm.current_position())}")
                elif cmd == "home":
                    self.go_home()
                else:
                    self.move_to(*self._three(nums), relative=(cmd == "rel"),
                                 execute=not self.plan_only, ask=self._ask)
                    if self.plan_only:
                        print("  not executed (plan-only mode)")
            except TargetRejected as e:
                print(f"  REJECTED, nothing sent: {e}")
            except (ValueError, PlanningFailed, RuntimeError, TimeoutError) as e:
                print(f"  failed: {e}")
            except KeyboardInterrupt:
                print("  interrupted")

    def _ask(self):
        if not self.table_on:
            print("  WARNING: the table is not in the planning scene")
        return confirm()

    @staticmethod
    def _three(nums):
        if len(nums) != 3:
            raise ValueError("need exactly three numbers: x y z")
        return nums


def load_config(path):
    with open(path) as f:
        cfg = yaml.safe_load(f)
    validate_safe_box(cfg["safe_box"])
    return cfg


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=str(HERE / "config.yaml"))
    parser.add_argument("--plan-only", action="store_true", help="plan and print, never move the arm")
    args = parser.parse_args()

    cfg = load_config(args.config)
    rclpy.init()
    arm = ArmClient(cfg)
    try:
        print("waiting for MoveIt...")
        arm.wait_ready()
        Session(arm, cfg, args.plan_only).run()
    except (TimeoutError, PlanningFailed, RuntimeError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    finally:
        arm.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

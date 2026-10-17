# openarmx-arm-control

Move the OpenArmX **right** gripper to a typed x, y, z position, through MoveIt,
from a terminal on the arm PC (`Y`, reached through the CloudLab terminal).

- Plans first and prints the plan; moves only after you type `y`, and runs exactly the plan shown.
- Keeps roughly the home gripper orientation (±0.3 rad); falls back to position-only and says so.
- Refuses targets outside a safe box around the home gripper position, and moves longer than 0.15 m.
- Compares several plans and keeps the one with the smallest joint motion; refuses it if any joint
  would still move more than 0.6 rad (the 7-joint arm can otherwise swing its elbow a lot for a small nudge).
- Adds the table to MoveIt's planning scene once its height is measured.
- After each move, prints where the gripper actually ended up and the error.

Nothing here touches CloudLab. Never run the motor-manager scripts while MoveIt is running.

## One-time setup on `Y`

```bash
git clone https://github.com/brucezhangcy/openarmx-arm-control.git ~/arm_control
```

Update later with `cd ~/arm_control && git pull`.

## Every session

1. Arm powered and **near zero** (the launch refuses to start if any joint is more than 30° off).
   Someone at the e-stop for anything that moves the real arm.
2. Start MoveIt in `tmux`, so a dropped browser tab doesn't kill it:

   ```bash
   tmux new -s moveit
   source /opt/ros/humble/setup.bash && source ~/openarmx_ws/install/setup.bash
   ros2 launch openarmx_bimanual_moveit_config demo.launch.py can_fd:=false
   # simulated hardware instead (rung 1):  ... demo.launch.py use_fake_hardware:=true
   ```

   RViz will fail to open (no screen); the rest keeps running. Detach with `Ctrl+B` then `D`.
3. In a second terminal:

   ```bash
   source /opt/ros/humble/setup.bash && source ~/openarmx_ws/install/setup.bash
   cd ~/arm_control && python3 ee_goto.py            # add --plan-only to never move
   ```

   ```
   target> pos                 current gripper position
   target> rel 0 0 0.03        3 cm up from where it is
   target> 0.30 -0.20 0.45     absolute position in the world frame (metres)
   target> home                back to all-zero joints
   target> quit
   ```

4. **Ending:** `home` → tell the on-site person → `tmux attach -t moveit` and `Ctrl+C`
   (stopping the stack disables the motors, so the arm settles from home) →
   check with the motor manager's `check_motor_status.py`.

Emergencies: the e-stop. Ctrl+C during a move asks MoveIt to stop the motion.

## Test ladder

| Rung | Run | Arm moves | Pass when |
|---|---|---|---|
| 1 | Stack with `use_fake_hardware:=true`; script plans and executes | No | Startup prints positions; bad targets are REJECTED; small moves plan and "execute" |
| 2 | Real stack; `ee_goto.py --plan-only` | Holds only | Printed position matches the real gripper; nudges plan |
| 3 | Real; `rel` moves of 0.02–0.03 m, one axis at a time | A little | Each goes the expected way; note the error numbers |
| 4 | Real; 3–4 absolute points inside the box, then `home`, twice | Yes | Repeatable; error within the agreed mark (~2 cm) |

Before rung 4, set `safe_box` and `table` in `config.yaml` from rung 2–3 measurements and set `table.enabled: true`.

## Troubleshooting

- **`/move_action not available`**: the MoveIt stack isn't running, or the terminal hasn't sourced the workspace.
- **Every plan fails with the home orientation**: try a target closer to the current position; the script already retries position-only.
- **MoveIt rejects `openarmx_right_hand_tcp`** (planning fails with a link/constraint error even for tiny moves):
  set `tip_link: openarmx_right_link7` in `config.yaml`. Targets then refer to the wrist flange,
  which is 0.18 m behind the gripper point along the gripper's axis.
- **`check_target` rejects everything**: `safe_box` is relative to the *home* gripper position printed at startup, not to the current one.

## Files

- `ee_goto.py`: the ROS 2 client and the interactive loop (`Session.move_to` is the reusable entry point).
- `safety.py`: target checks and command parsing, no ROS imports. Tests: `python3 -m unittest test_safety`.
- `config.yaml`: frames, speeds, tolerances, safe box, table. Values marked MEASURE come from the real arm.

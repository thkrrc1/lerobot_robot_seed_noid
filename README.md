# lerobot_robot_seed_noid

External LeRobot `Robot` plugin for the SEED/NOID ROS 2 Jazzy + ros2_control stack.
It is installed locally and does **not** require a pull request to the LeRobot GitHub repository.

## Architecture

```text
LeRobot CLI / rollout / async / RTC
            |
      SeedNoid Robot plugin
            |
  ROS 2 topics/actions/services
            |
 ros2_control controllers / hardware
```

The plugin exposes the existing 24-axis feature order to LeRobot and distributes
commands internally to multiple JointTrajectoryController endpoints.

## What changed from the custom Bridge

The plugin does not contain its own policy-loading or inference loop. LeRobot handles:

- checkpoint/policy loading
- preprocessor and postprocessor pipelines
- synchronous rollout
- RTC rollout
- asynchronous policy server/client execution
- dataset recording

The plugin handles only robot I/O, feature ordering, controller routing, and safety.
See `MIGRATION.md`.

## Installation

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2/ros2_ws7/install/setup.bash

# The plugin does not require the Python cv_bridge binding for sensor_msgs/Image topics.
python3 -m venv --system-site-packages ~/venvs/lerobot_seed_noid
source ~/venvs/lerobot_seed_noid/bin/activate

# Install the desired LeRobot extras in the same environment.
pip install 'lerobot[core_scripts]'
# For the separate async PolicyServer/RobotClient path:
pip install 'lerobot[async]'

cd /path/to/lerobot_robot_seed_noid
pip install -e .
```

No upload to PyPI or GitHub is required. `pip install -e .` registers the external
package in the local Python environment, and LeRobot discovers the
`lerobot_robot_` package prefix automatically.

## Verify plugin discovery without connecting ROS 2

```bash
python examples/check_plugin.py
```

Expected key points:

- robot class: `SeedNoid`
- `action dimension: 24`
- default `command_groups: ['rarm']`

## Observe the ROS 2 robot without sending commands

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2/ros2_ws7/install/setup.bash
python examples/read_observation.py
```

## Existing 24-axis order

```text
0..6   rarm
7      rhand / r_thumb_joint
8..14  larm
15     lhand / l_thumb_joint
16..18 waist
19..20 lifter
21..23 head
```

`action_groups` defines this LeRobot feature contract. `command_groups` is a
separate safety gate. This allows an existing 24-axis checkpoint to pass feature
compatibility checks while initially commanding only the right arm.

## Base synchronous rollout

```bash
lerobot-rollout \
  --strategy.type=base \
  --policy.path=/home/seed/learning_output/output_lerobot/pi0_robot_mouse_pick_place_100steps/checkpoints/last/pretrained_model \
  --robot.type=seed_noid \
  --robot.id=seed_noid_pi0 \
  --robot.joint_state_topic=/joint_states \
  --robot.joint_command_transport=topic \
  --robot.trajectory_duration_s=0.6 \
  --robot.action_groups='[rarm,rhand,larm,lhand,waist,lifter,head]' \
  --robot.command_groups='[rarm]' \
  --robot.max_relative_target=0.04 \
  --robot.ros_image_topics='{camera1: /camera/camera/color/image_raw}' \
  --robot.ros_image_shapes='{camera1: [720,1280,3]}' \
  --task='pick up the mouse from the desk, lift it, lower it, and place it back on the desk' \
  --duration=120 \
  --device=cuda
```

Confirm the exact `lerobot-rollout --help` spelling for the installed LeRobot
version because the rollout CLI is evolving.

## RTC rollout

RTC is provided by LeRobot, not by this package.

```bash
lerobot-rollout \
  --strategy.type=base \
  --policy.path=/path/to/checkpoints/last/pretrained_model \
  --inference.type=rtc \
  --inference.rtc.execution_horizon=10 \
  --inference.rtc.max_guidance_weight=10.0 \
  --robot.type=seed_noid \
  --robot.id=seed_noid_pi0_rtc \
  --robot.joint_command_transport=topic \
  --robot.action_groups='[rarm,rhand,larm,lhand,waist,lifter,head]' \
  --robot.command_groups='[rarm]' \
  --robot.max_relative_target=0.04 \
  --robot.ros_image_topics='{camera1: /camera/camera/color/image_raw}' \
  --robot.ros_image_shapes='{camera1: [720,1280,3]}' \
  --task='pick up the mouse from the desk, lift it, lower it, and place it back on the desk' \
  --duration=120 \
  --device=cuda
```

RTC only applies to policies supported by the installed LeRobot version.

## LeRobot async PolicyServer / RobotClient

Terminal 1:

```bash
python -m lerobot.async_inference.policy_server --host=127.0.0.1 --port=8080
```

Terminal 2:

```bash
python -m lerobot.async_inference.robot_client \
  --server_address=127.0.0.1:8080 \
  --robot.type=seed_noid \
  --robot.id=seed_noid_async \
  --robot.joint_command_transport=topic \
  --robot.action_groups='[rarm,rhand,larm,lhand,waist,lifter,head]' \
  --robot.command_groups='[rarm]' \
  --robot.max_relative_target=0.04 \
  --robot.ros_image_topics='{camera1: /camera/camera/color/image_raw}' \
  --robot.ros_image_shapes='{camera1: [720,1280,3]}' \
  --task='pick up the mouse from the desk' \
  --policy_type=pi0 \
  --pretrained_name_or_path=/path/to/checkpoints/last/pretrained_model \
  --policy_device=cuda \
  --actions_per_chunk=50
```

Use the async command names shown by the installed version's `--help`; the
server/client interface can change between LeRobot releases.

## Dataset recording

The Robot plugin is sufficient for rollout, RTC, and the async robot client.
`lerobot-record` additionally requires a LeRobot `Teleoperator` plugin whose
action feature names match this robot. Until that teleoperator exists, keep using
the ROSBag converter under `legacy_tools/` for demonstrations.

Once a compatible teleoperator is installed:

```bash
lerobot-record \
  --robot.type=seed_noid \
  --robot.id=seed_noid_record \
  --robot.action_groups='[rarm,rhand,larm,lhand,waist,lifter,head]' \
  --robot.command_groups='[rarm]' \
  --robot.ros_image_topics='{camera1: /camera/camera/color/image_raw}' \
  --robot.ros_image_shapes='{camera1: [720,1280,3]}' \
  --teleop.type=<compatible_teleoperator> \
  --dataset.repo_id=local/seed_noid_dataset \
  --dataset.root=/home/seed/lerobot_datasets/seed_noid_dataset \
  --dataset.fps=10 \
  --dataset.num_episodes=10 \
  --dataset.single_task='pick up the mouse'
```

## Hand command modes

For each hand:

- `joint_trajectory`: send the thumb action to the hand controller.
- `script_service`: convert the existing thumb action value to the Aero
  `RunScript` grasp/release service using thresholds.
- `disabled`: retain the action feature but do not command the hand.

Example:

```bash
--robot.command_groups='[rarm,rhand]' \
--robot.hand_command_modes='{rhand: script_service, lhand: disabled}' \
--robot.hand_grasp_threshold=0.75 \
--robot.hand_release_threshold=0.25
```

## Multiple controllers

LeRobot sees one ordered action vector. `SeedNoid.send_action()` distributes it
to the configured ROS 2 controllers. Controller count does not change the
LeRobot dataset or policy interface.

## Safety notes

- Default feature dimension is 24, but only `rarm` is commanded by default.
- Start with `command_groups=[rarm]` and add one group at a time.
- Keep `max_relative_target` enabled during initial tests.
- Populate `joint_position_limits` from the actual URDF/controller limits.
- Verify action/state order against the checkpoint metadata before motion.

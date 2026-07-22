# Migration from the custom ROS2 model bridge

## Removed from the primary runtime

The following custom runtime responsibilities are intentionally not included in this plugin:

- Loading policies with `PI0Policy.from_pretrained()` / `ACTPolicy.from_pretrained()`
- Calling `make_pre_post_processors()` directly
- The custom `AsyncPolicyRunner` thread and action queue
- Custom chunk blending/prefill loops
- A separate `run_lerobot_model_to_ros2_bridge.py` control loop

LeRobot now owns those responsibilities through:

- `lerobot-rollout` for policy deployment
- `--inference.type=rtc` for Real-Time Chunking
- `lerobot.async_inference.policy_server` and `robot_client` for async inference
- `lerobot-record` for direct LeRobotDataset recording when a compatible teleoperator is available

## Retained in the Robot plugin

- `/joint_states` and ROS image subscriptions
- Fixed 24-axis state/action ordering
- Distribution of one LeRobot action across multiple ROS 2 controllers
- Joint position and per-step relative safety limits
- Optional hand `RunScript` service conversion
- Optional base Twist commands

## ROSBag converter

The previous ROSBag converter is retained under `legacy_tools/` as a migration and debugging fallback. It remains useful until a LeRobot-compatible teleoperator plugin is available for direct `lerobot-record` demonstration collection.

## Intentionally omitted from the primary plugin

Generic vector-valued ROS topics and TF-to-vector observations are not exposed directly in this first plugin version. Current LeRobot hardware feature conversion treats scalar numeric keys and image shapes specially; arbitrary vector observations should be added through a dedicated LeRobot RobotProcessor pipeline so dataset and policy feature metadata remain correct. The original decoders remain in `legacy_tools/`.

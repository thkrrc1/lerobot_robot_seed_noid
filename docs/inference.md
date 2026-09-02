# Async Inference

学習済み π0 Checkpoint を使って SEED-Noid を実機推論します。

## 構成

```text
Robot PC
  ROS 2
  SEED-Noid Plugin
  RobotClient
      │
      │ LAN
      ▼
Inference PC
  PolicyServer
  π0 Checkpoint
  GPU
```

PolicyServer と RobotClient は同一PCでも別PCでも実行できます。

## Robot PC の準備

```bash
export ROBOT_WS=~/ros2/<workspace>

source /opt/ros/jazzy/setup.bash
source "${ROBOT_WS}/install/setup.bash"
source ~/venvs/lerobot_v060/bin/activate
```

Topic確認：

```bash
ros2 topic echo /joint_states --once

ros2 topic hz /camera1/image_raw/compressed
ros2 topic hz /camera2/image_raw/compressed
ros2 topic hz /camera3/image_raw/compressed
```

## PolicyServer

Inference PC：

```bash
source ~/venvs/lerobot_v060/bin/activate
```

LAN から接続する場合：

```bash
python -m lerobot.async_inference.policy_server \
  --host=0.0.0.0 \
  --port=8080
```

同一PCのみの場合は `127.0.0.1` でも構いません。

## RobotClient

Checkpoint：

```bash
export CHECKPOINT=~/learning_outputs/pi0_seed_noid_pipeline_test/checkpoints/last/pretrained_model
```

Inference PC が `192.168.0.60` の例：

```bash
python -m lerobot.async_inference.robot_client \
  --server_address=192.168.0.60:8080 \
  --robot.type=seed_noid \
  --robot.command_groups='["rarm","rhand","larm","lhand","waist","lifter","head"]' \
  --task="Take the snacks off the shelf and move them up one shelf." \
  --policy_type=pi0 \
  --pretrained_name_or_path="${CHECKPOINT}" \
  --policy_device=cuda \
  --client_device=cpu \
  --actions_per_chunk=50 \
  --chunk_size_threshold=0.5 \
  --aggregate_fn_name=weighted_average \
  --debug_visualize_queue_size=True
```

## Camera

`config_seed_noid.py` のデフォルト：

```text
head  → /camera1/image_raw/compressed → 480 x 640 x 3
right → /camera2/image_raw/compressed → 480 x 640 x 3
left  → /camera3/image_raw/compressed → 480 x 640 x 3
```

そのため、現在の構成では `robot_client` で Camera 引数を省略できます。

明示的に上書きする場合：

```bash
--robot.ros_image_topics='{"head":"/camera1/image_raw/compressed","right":"/camera2/image_raw/compressed","left":"/camera3/image_raw/compressed"}' \
--robot.ros_image_shapes='{"head":[480,640,3],"right":[480,640,3],"left":[480,640,3]}'
```

## command_groups

全Group：

```bash
--robot.command_groups='["rarm","rhand","larm","lhand","waist","lifter","head"]'
```

初回確認で右腕だけ：

```bash
--robot.command_groups='["rarm"]'
```

## `No fresh ROS image received`

確認：

```bash
ros2 topic hz /camera1/image_raw/compressed
ros2 topic hz /camera2/image_raw/compressed
ros2 topic hz /camera3/image_raw/compressed
```

以下を確認します。

- Camera Node が起動している
- Topic 名が一致している
- `CompressedImage` が配信されている
- Camera 名が `head/right/left`
- Shape が `480 x 640 x 3`

## `All image features are missing from the batch`

Checkpoint が要求する：

```text
observation.images.head
observation.images.right
observation.images.left
```

と RobotClient から送る Camera feature が一致しているか確認します。

## 実行前チェック

```bash
python -c "import lerobot_robot_seed_noid; print('Plugin OK')"

ros2 topic echo /joint_states --once

ros2 topic list | grep -Ei "camera|image"

python - <<'PY'
import torch
print("CUDA available:", torch.cuda.is_available())
PY
```

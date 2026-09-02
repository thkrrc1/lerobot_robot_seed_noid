# ROS 2 Bag → LeRobot Dataset

このディレクトリは、ROS 2 Bag を LeRobot Dataset へ変換するためのツールです。

```text
dataset_tools/
├── rosbag_to_lerobot_dataset.py
├── runtime_config.py
└── topic_decoders.py
```

## 前提

ROS 2 Jazzy の Python package を利用するため、ROS 2 環境を読み込みます。

```bash
export ROBOT_WS=~/ros2/<workspace>

source /opt/ros/jazzy/setup.bash
source "${ROBOT_WS}/install/setup.bash"
source ~/venvs/lerobot_v060/bin/activate
```

## Dataset Contract

State / Action：

```text
rarm → rhand → larm → lhand → waist → lifter → head
```

合計24軸です。

Camera：

```text
head  → /camera1/image_raw/compressed
right → /camera2/image_raw/compressed
left  → /camera3/image_raw/compressed
```

Dataset feature：

```text
observation.state
action
observation.images.head
observation.images.right
observation.images.left
task
```

## ROS 2 Bag の確認

```bash
ros2 bag info ~/rosbag/<episode>
```

主な対象 Topic：

```text
/joint_states

/camera1/image_raw/compressed
/camera2/image_raw/compressed
/camera3/image_raw/compressed

/rarm_controller/joint_trajectory
/rhand_controller/joint_trajectory
/larm_controller/joint_trajectory
/lhand_controller/joint_trajectory
/waist_controller/joint_trajectory
/lifter_controller/joint_trajectory
/head_controller/joint_trajectory
```

1つの merged ROS 2 Bag directory を 1 episode として扱います。

## inspect-only

まず Dataset を書き出さず、Bag の読み込み・Topic・同期状態を確認します。

```bash
python dataset_tools/rosbag_to_lerobot_dataset.py \
  --config-file configs/dataset_config.yaml \
  --bag \
    ~/rosbag/snack_ros2bag/1 \
    ~/rosbag/snack_ros2bag/2 \
    ~/rosbag/snack_ros2bag/3 \
    ~/rosbag/snack_ros2bag/4 \
    ~/rosbag/snack_ros2bag/5 \
  --inspect-only
```

## Dataset 作成

```bash
export DATASET_ROOT=~/lerobot_datasets/seed_noid
```

```bash
python dataset_tools/rosbag_to_lerobot_dataset.py \
  --config-file configs/dataset_config.yaml \
  --bag \
    ~/rosbag/snack_ros2bag/1 \
    ~/rosbag/snack_ros2bag/2 \
    ~/rosbag/snack_ros2bag/3 \
    ~/rosbag/snack_ros2bag/4 \
    ~/rosbag/snack_ros2bag/5 \
  --repo-id local/seed_noid \
  --root "${DATASET_ROOT}" \
  --overwrite
```

`--overwrite` を付けると既存 Dataset を作り直します。

## Dataset 確認

```bash
python - <<'PY'
import os
from lerobot.datasets import LeRobotDataset

root = os.path.expanduser("~/lerobot_datasets/seed_noid")

ds = LeRobotDataset(
    "local/seed_noid",
    root=root,
)

print("frames:", len(ds))
print(ds[0]["observation.state"].shape)
print(ds[0]["action"].shape)

for key in (
    "observation.images.head",
    "observation.images.right",
    "observation.images.left",
):
    print(key, ds[0][key].shape)
PY
```

期待：

```text
observation.state : 24
action            : 24
```

Camera は3系統が存在することを確認します。

## 注意

一度学習を開始した Dataset については、以下を変更しないでください。

- Joint 順序
- Action 順序
- Camera feature 名
- Camera の意味対応

学習と推論の Feature Contract が一致しなくなるためです。

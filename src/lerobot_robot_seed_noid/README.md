# SEED-Noid Robot Plugin

このディレクトリは、SEED-Noid を LeRobot の `Robot` として扱うための Plugin 実装です。

主なファイル：

```text
lerobot_robot_seed_noid/
├── __init__.py
├── config_seed_noid.py
└── seed_noid.py
```

## 役割

`config_seed_noid.py`：

- 24軸の Joint Group
- Observation / Action の順序
- ros2_control Controller 名
- Camera Topic / Shape
- Action transport
- Safety 設定

`seed_noid.py`：

- `/joint_states` の購読
- ROS Image / CompressedImage の購読
- LeRobot Observation の生成
- JointTrajectory の送信
- Action の安全制限
- RobotClient / record / rollout から利用する Robot I/O

## インストール

リポジトリルートで：

```bash
source ~/venvs/lerobot_v060/bin/activate
python -m pip install -e .
```

確認：

```bash
python -c "import lerobot_robot_seed_noid; print('OK')"
```

## ROS 2 環境

```bash
export ROBOT_WS=~/ros2/<workspace>

source /opt/ros/jazzy/setup.bash
source "${ROBOT_WS}/install/setup.bash"
source ~/venvs/lerobot_v060/bin/activate
```

venv は ROS 2 Python package を参照できるよう、`--system-site-packages` 付きで作成してください。

## Joint Contract

```text
rarm:
  r_shoulder_p_joint
  r_shoulder_r_joint
  r_shoulder_y_joint
  r_elbow_joint
  r_wrist_y_joint
  r_wrist_p_joint
  r_wrist_r_joint

rhand:
  r_thumb_joint

larm:
  l_shoulder_p_joint
  l_shoulder_r_joint
  l_shoulder_y_joint
  l_elbow_joint
  l_wrist_y_joint
  l_wrist_p_joint
  l_wrist_r_joint

lhand:
  l_thumb_joint

waist:
  waist_y_joint
  waist_p_joint
  waist_r_joint

lifter:
  knee_joint
  ankle_joint

head:
  neck_y_joint
  neck_p_joint
  neck_r_joint
```

順序：

```text
rarm → rhand → larm → lhand → waist → lifter → head
```

## Camera

デフォルト：

```text
head  → /camera1/image_raw/compressed
right → /camera2/image_raw/compressed
left  → /camera3/image_raw/compressed
```

Shape：

```text
head  = (480, 640, 3)
right = (480, 640, 3)
left  = (480, 640, 3)
```

Plugin では Camera 名をそのまま LeRobot observation の key として使用します。

```text
head
right
left
```

LeRobot 側では最終的に：

```text
observation.images.head
observation.images.right
observation.images.left
```

として扱われます。

## Action Command

デフォルト transport：

```text
topic
```

各 Group は以下の形式で `JointTrajectory` を送信します。

```text
/<controller_name>/joint_trajectory
```

例：

```text
/rarm_controller/joint_trajectory
/rhand_controller/joint_trajectory
/larm_controller/joint_trajectory
/lhand_controller/joint_trajectory
/waist_controller/joint_trajectory
/lifter_controller/joint_trajectory
/head_controller/joint_trajectory
```

## command_groups

`action_groups` は Policy の Feature Contract です。

`command_groups` は実際に Robot へ指令を送る Group を制御する Safety Gate です。

全Group：

```bash
--robot.command_groups='["rarm","rhand","larm","lhand","waist","lifter","head"]'
```

動作確認時に右腕だけへ限定する例：

```bash
--robot.command_groups='["rarm"]'
```

この場合でも Policy の24軸 Action Contract 自体は維持されます。

## max_relative_target

デフォルト：

```text
0.04 rad
```

現在 Joint 値から1回の送信で大きく離れた Target が来た場合に、相対移動量を制限します。

これは学習時の `--steps` とは別の設定です。

## Camera の確認

```bash
ros2 topic hz /camera1/image_raw/compressed
ros2 topic hz /camera2/image_raw/compressed
ros2 topic hz /camera3/image_raw/compressed
```

`No fresh ROS image received` が出る場合は、Topic 名、配信状態、Shape、Camera 名を確認してください。

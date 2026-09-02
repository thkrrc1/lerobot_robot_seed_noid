# LeRobot + SEED-Noid

SEED-Noid を LeRobot から利用するための Robot Plugin と、ROS 2 Bag から LeRobot Dataset を作成するための変換ツールです。

## 全体フロー

```text
ROS 2 / SEED-Noid
        │
        ├─ ROS 2 Bag
        │      │
        │      ▼
        │  Dataset Converter
        │      │
        │      ▼
        │  LeRobot Dataset
        │      │
        │      ▼
        │     π0 学習
        │      │
        │      ▼
        │   Checkpoint
        │
        └─ Robot Plugin
               │
               ▼
          RobotClient
               │
               ▼
          PolicyServer
               │
               ▼
           SEED-Noid
```

## ドキュメント

- [Robot Plugin](src/lerobot_robot_seed_noid/README.md)
- [ROS 2 Bag → LeRobot Dataset](dataset_tools/README.md)
- [設定ファイル](configs/README.md)
- [π0 学習](docs/training.md)
- [Async 推論](docs/inference.md)

## 前提

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12
- LeRobot v0.6.0 系
- NVIDIA GPU / CUDA（π0 学習・GPU 推論時）

ROS 2 workspace は環境ごとに異なるため、以下のように設定します。

```bash
export ROBOT_WS=~/ros2/<workspace>
```

ROS 2 を使うターミナルでは次を実行します。

```bash
source /opt/ros/jazzy/setup.bash
<<<<<<< HEAD
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
=======
source "${ROBOT_WS}/install/setup.bash"
>>>>>>> c4dcac6 (Refactoring plugin)
```

## Python 環境

ROS 2 の Python package を venv から利用するため、`--system-site-packages` を付けます。

```bash
python3.12 -m venv \
  --system-site-packages \
  ~/venvs/lerobot_v060

source ~/venvs/lerobot_v060/bin/activate

python -m pip install --upgrade pip setuptools wheel
python -m pip install -r lerobot_v060_requirements.txt
python -m pip install -e .
```

依存関係確認：

```bash
python -m pip check
```

## 共通 Feature Contract

### State / Action

全軸を以下の順序で使用します。

```text
rarm(7)
→ rhand(1)
→ larm(7)
→ lhand(1)
→ waist(3)
→ lifter(2)
→ head(3)
```

合計：

```text
all axis
```

Dataset 作成後にこの順序を変更しないでください。

### Camera

```text
head  → /camera1/image_raw/compressed → 480 x 640 x 3
right → /camera2/image_raw/compressed → 480 x 640 x 3
left  → /camera3/image_raw/compressed → 480 x 640 x 3
```

LeRobot Dataset 上では以下になります。

```text
observation.images.head
observation.images.right
observation.images.left
```

Dataset、学習済み Checkpoint、推論で同じ Camera 名を使用してください。

## 最短の確認

Plugin：

```bash
python -c "import lerobot_robot_seed_noid; print('OK')"
```

ROS 2：

```bash
ros2 topic echo /joint_states --once
ros2 topic list | grep -Ei "camera|image"
```

Dataset：

```bash
python dataset_tools/rosbag_to_lerobot_dataset.py \
  --config-file configs/dataset_config.yaml \
  --bag ~/rosbag/<episode> \
  --inspect-only
```

詳細は各ドキュメントを参照してください。

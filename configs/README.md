# Config Files

このディレクトリでは Dataset 作成・Robot 実行に使用する設定を管理します。

推奨構成：

```text
configs/
├── dataset_config.yaml
└── learning_config.yaml
```

## dataset_config.yaml

ROS 2 Bag → LeRobot Dataset 変換用です。

主な設定：

```yaml
dataset:
  storage_id: mcap
  repo_id: local/seed_noid
  robot_type: seed_noid
  fps: 30
```

Camera：

```yaml
topics:
  observation:
    cameras:
      head:
        topic: /camera1/image_raw/compressed
        height: 480
        width: 640

      right:
        topic: /camera2/image_raw/compressed
        height: 480
        width: 640

      left:
        topic: /camera3/image_raw/compressed
        height: 480
        width: 640
```

State / Action Group：

```yaml
state_order_groups: [rarm, rhand, larm, lhand, waist, lifter, head]
action_order_groups: [rarm, rhand, larm, lhand, waist, lifter, head]
```

この順序は学習開始後に変更しないでください。

## learning_config.yaml

Robot / 推論 CLI の設定値を整理するための参照ファイルです。

例：

```yaml
robot:
  type: seed_noid
  id: seed_noid
  ros_node_name: lerobot_seed_noid
  joint_state_topic: /joint_states
  joint_command_transport: topic
  trajectory_duration_s: 0.6

  observation_groups: [rarm, rhand, larm, lhand, waist, lifter, head]
  action_groups: [rarm, rhand, larm, lhand, waist, lifter, head]

  command_groups: [rarm, rhand, larm, lhand, waist, lifter, head]
  max_relative_target: 0.04

  ros_image_topics:
    head: /camera1/image_raw/compressed
    right: /camera2/image_raw/compressed
    left: /camera3/image_raw/compressed

  ros_image_shapes:
    head: [480, 640, 3]
    right: [480, 640, 3]
    left: [480, 640, 3]

  hand_command_modes:
    rhand: joint_trajectory
    lhand: joint_trajectory
```

## learning_config.yaml の扱い

現在の `robot_client` 実行例では `learning_config.yaml` を直接指定していません。

そのため推論時の設定は主に：

1. `config_seed_noid.py` のデフォルト値
2. `--robot.*` CLI 引数

で決まります。

`learning_config.yaml` は、使用する値を明示・共有するための参照設定として扱います。

## Path の扱い

Git 管理する YAML に `/home/<user>/...` の固定Pathを極力入れない運用を推奨します。

実行時は Shell 側で：

```bash
export DATASET_ROOT=~/lerobot_datasets/seed_noid
```

として、

```bash
--root "${DATASET_ROOT}"
```

のように CLI で上書きするとユーザー名に依存しません。

YAML 内の `~` は Shell と同じように必ず展開されるとは限らないため注意してください。

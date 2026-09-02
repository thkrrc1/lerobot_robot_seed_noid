# π0 Training

ROS 2 Bag から作成した LeRobot Dataset を使用して π0 を追加学習します。

## 前提

Dataset：

```text
repo_id: local/seed_noid
root: ~/lerobot_datasets/seed_noid
```

Feature Contract：

```text
observation.state = 24
action            = 24

observation.images.head
observation.images.right
observation.images.left
```

## 環境

```bash
source ~/venvs/lerobot_v060/bin/activate
```

```bash
export DATASET_ROOT=~/lerobot_datasets/seed_noid
export OUTPUT_DIR=~/learning_outputs/pi0_seed_noid_pipeline_test
```

## 学習例

```bash
lerobot-train \
  --dataset.repo_id=local/seed_noid \
  --dataset.root="${DATASET_ROOT}" \
  --policy.type=pi0 \
  --policy.pretrained_path=lerobot/pi0_base \
  --output_dir="${OUTPUT_DIR}" \
  --job_name=pi0_seed_noid_pipeline_test \
  --policy.gradient_checkpointing=true \
  --policy.dtype=bfloat16 \
  --policy.device=cuda \
  --batch_size=8 \
  --steps=3000
```

`batch_size`、`steps` は Dataset 数と GPU メモリに応じて調整してください。

## Checkpoint

例：

```text
~/learning_outputs/
└── pi0_seed_noid_pipeline_test/
    └── checkpoints/
        └── last/
            └── pretrained_model/
```

```bash
export CHECKPOINT=~/learning_outputs/pi0_seed_noid_pipeline_test/checkpoints/last/pretrained_model
```

## 学習前に確認するもの

### State

```text
24 axis
rarm → rhand → larm → lhand → waist → lifter → head
```

### Action

```text
24 axis
rarm → rhand → larm → lhand → waist → lifter → head
```

### Camera

```text
observation.images.head
observation.images.right
observation.images.left
```

これらは推論時も同じにしてください。

## 注意

`--steps` は学習更新回数です。

Robot Plugin の：

```text
max_relative_target
```

とは別の設定なので、`--steps` を増減したからといって `max_relative_target` を比例変更する必要はありません。

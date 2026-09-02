#!/usr/bin/env python3
"""Configurable ROS 2 bag -> LeRobotDataset converter.

Default behavior uses the SEED Noid  dataset contract:
  image feature: observation.images.camera1
  image topic:   /camera/camera/color/image_raw
  state/action:  rarm,rhand,larm,lhand,waist,lifter,head joint order

The converter now supports a variable number of cameras, controller action
topics, joint groups, and state/action dimensions. The important rule is that
training-time feature names must match inference-time feature names exactly:
  --camera front=/camera/front/image_raw,480,640
creates LeRobot feature: observation.images.front
"""

from __future__ import annotations

import argparse
import bisect
import inspect
import json
import shutil
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except Exception as exc:  # pragma: no cover
    print("ERROR: ROS 2 Python packages are required: rosbag2_py, rclpy, rosidl_runtime_py", file=sys.stderr)
    print(f"Import error: {exc}", file=sys.stderr)
    raise

# This converter lives under legacy_tools/. Add that directory and the project
# root to sys.path so it works both from an editable install and when invoked
# directly from the checked-out package.
_THIS_FILE = Path(__file__).resolve()
_TOOLS_DIR = _THIS_FILE.parent
_PROJECT_ROOT = _TOOLS_DIR.parent
for _path in (str(_TOOLS_DIR), str(_PROJECT_ROOT)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from runtime_config import (  # type: ignore
    RosCameraSpec,
    joint_order_from_groups,
    load_mapping,
    load_yaml_or_json_config,
    merge_joint_groups,
    merge_string_maps,
    parse_assignment_list,
    parse_camera_specs,
    parse_csv_list,
    prefixed_image_key,
    rosbag_config_to_argparse_defaults,
)
from topic_decoders import (  # type: ignore
    decode_numeric_message,
    decode_tf_message,
    names_for_spec,
    shape_for_spec,
)

# Canonical feature contract for SEED Noid.
# Keep this converter independent of the currently enabled safety groups in
# config_seed_noid.py: the runtime plugin may expose/command only a subset,
# while the dataset contract intentionally contains all
DEFAULT_JOINT_GROUPS = {
    "rarm": [
        "r_shoulder_p_joint",
        "r_shoulder_r_joint",
        "r_shoulder_y_joint",
        "r_elbow_joint",
        "r_wrist_y_joint",
        "r_wrist_p_joint",
        "r_wrist_r_joint",
    ],
    "rhand": ["r_thumb_joint"],
    "larm": [
        "l_shoulder_p_joint",
        "l_shoulder_r_joint",
        "l_shoulder_y_joint",
        "l_elbow_joint",
        "l_wrist_y_joint",
        "l_wrist_p_joint",
        "l_wrist_r_joint",
    ],
    "lhand": ["l_thumb_joint"],
    "waist": ["waist_y_joint", "waist_p_joint", "waist_r_joint"],
    "lifter": ["knee_joint", "ankle_joint"],
    "head": ["neck_y_joint", "neck_p_joint", "neck_r_joint"],
}

DEFAULT_CONTROLLER_NAMES = {
    "rarm": "rarm_controller",
    "rhand": "rhand_controller",
    "larm": "larm_controller",
    "lhand": "lhand_controller",
    "waist": "waist_controller",
    "lifter": "lifter_controller",
    "head": "head_controller",
}


DEFAULT_IMAGE_TOPIC = "/camera1/image_raw/compressed"
DEFAULT_JOINT_STATES_TOPIC = "/joint_states"
DEFAULT_TASK = "Take the snacks off the shelf and move them up one shelf."
DEFAULT_POLICY_GROUP_ORDER = ["rarm", "rhand", "larm", "lhand", "waist", "lifter", "head"]
DEFAULT_ACTION_TOPICS = {
    "rarm": "/rarm_controller/joint_trajectory",
    "rhand": "/rhand_controller/joint_trajectory",
    "larm": "/larm_controller/joint_trajectory",
    "lhand": "/lhand_controller/joint_trajectory",
    "waist": "/waist_controller/joint_trajectory",
    "lifter": "/lifter_controller/joint_trajectory",
    "head": "/head_controller/joint_trajectory",
}


@dataclass
class BagData:
    images: dict[str, list[tuple[float, np.ndarray]]]
    states: list[tuple[float, np.ndarray]]
    action_groups: dict[str, list[tuple[float, np.ndarray]]]
    extra_observations: dict[str, list[tuple[float, np.ndarray]]]
    counts: dict[str, int]
    camera_topics: dict[str, str]
    action_topic_by_group: dict[str, str]


def detect_storage_id(bag_path: Path) -> str:
    """Best-effort storage id detection for sqlite3 vs mcap."""
    if (bag_path / "metadata.yaml").exists():
        text = (bag_path / "metadata.yaml").read_text(errors="ignore")
        for line in text.splitlines():
            if "storage_identifier:" in line:
                return line.split(":", 1)[1].strip()
    if list(bag_path.glob("*.mcap")):
        return "mcap"
    if list(bag_path.glob("*.db3")):
        return "sqlite3"
    return "sqlite3"


def bag_time_to_sec(t_ns: int) -> float:
    return float(t_ns) * 1e-9


def open_reader(bag_path: str | Path, storage_id: str | None = None) -> tuple[Any, dict[str, str]]:
    bag_path = Path(bag_path)
    if storage_id is None or storage_id == "auto":
        storage_id = detect_storage_id(bag_path)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    return reader, type_map


def get_series_times(series: list[tuple[float, Any]]) -> list[float]:
    return [x[0] for x in series]


def nearest_by_time(series: list[tuple[float, Any]], t: float, max_dt: float | None = None) -> Any | None:
    if not series:
        return None
    times = get_series_times(series)
    i = bisect.bisect_left(times, t)
    candidates = []
    if i > 0:
        candidates.append(series[i - 1])
    if i < len(series):
        candidates.append(series[i])
    if not candidates:
        return None
    best_t, best_v = min(candidates, key=lambda x: abs(x[0] - t))
    if max_dt is not None and abs(best_t - t) > max_dt:
        return None
    return best_v


def latest_before(series: list[tuple[float, Any]], t: float, max_age: float | None = None) -> Any | None:
    if not series:
        return None
    times = get_series_times(series)
    i = bisect.bisect_right(times, t) - 1
    if i < 0:
        return None
    ts, value = series[i]
    if max_age is not None and (t - ts) > max_age:
        return None
    return value


def future_after(series: list[tuple[float, Any]], t: float, future_sec: float, max_dt: float | None = None) -> Any | None:
    return nearest_by_time(series, t + future_sec, max_dt=max_dt)


def vector_from_joint_state(msg: Any, joint_order: list[str], strict: bool = True) -> np.ndarray | None:
    name_to_pos = dict(zip(list(msg.name), list(msg.position), strict=False))
    values: list[float] = []
    for joint_name in joint_order:
        if joint_name not in name_to_pos:
            if strict:
                raise KeyError(f"joint {joint_name!r} not found in /joint_states")
            return None
        values.append(float(name_to_pos[joint_name]))
    return np.asarray(values, dtype=np.float32)


def vector_from_joint_trajectory(msg: Any, joint_order: list[str], point_mode: str = "final") -> np.ndarray | None:
    if len(msg.points) == 0:
        return None
    if point_mode == "first":
        point = msg.points[0]
    elif point_mode == "final":
        point = msg.points[-1]
    else:
        raise ValueError(f"Unsupported point mode: {point_mode}")
    if len(point.positions) == 0:
        return None
    name_to_pos = dict(zip(list(msg.joint_names), list(point.positions), strict=False))
    values: list[float] = []
    for joint_name in joint_order:
        if joint_name not in name_to_pos:
            raise KeyError(f"joint {joint_name!r} not found in joint_trajectory.joint_names={list(msg.joint_names)}")
        values.append(float(name_to_pos[joint_name]))
    return np.asarray(values, dtype=np.float32)


def image_msg_to_rgb_array(msg: Any) -> np.ndarray:
    """Convert sensor_msgs/Image or sensor_msgs/CompressedImage to HWC RGB uint8."""
    # sensor_msgs/CompressedImage has ``format`` and ``data`` but no height/width.
    if hasattr(msg, "format") and not hasattr(msg, "encoding"):
        try:
            import cv2
        except Exception as exc:
            raise RuntimeError(
                "opencv-python is required to decode sensor_msgs/CompressedImage"
            ) from exc

        encoded = np.frombuffer(msg.data, dtype=np.uint8)
        bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError(f"Failed to decode CompressedImage format={getattr(msg, 'format', '')!r}")
        return np.ascontiguousarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), dtype=np.uint8)

    # sensor_msgs/Image
    try:
        from cv_bridge import CvBridge

        bridge = CvBridge()
        return np.ascontiguousarray(bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8"), dtype=np.uint8)
    except Exception:
        pass

    encoding = str(msg.encoding).lower()
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    channels = {
        "rgb8": 3,
        "8uc3": 3,
        "bgr8": 3,
        "rgba8": 4,
        "8uc4": 4,
        "bgra8": 4,
        "mono8": 1,
        "8uc1": 1,
    }.get(encoding)
    if channels is None:
        raise ValueError(f"Unsupported raw Image encoding: {msg.encoding}")

    required_row = w * channels
    if step < required_row:
        raise ValueError(f"Image.step={step} is smaller than required row bytes={required_row}")
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    required_size = h * step
    if buf.size < required_size:
        raise ValueError(f"Image data too short: got={buf.size}, required={required_size}")
    rows = buf[:required_size].reshape(h, step)
    arr = rows[:, :required_row].reshape(h, w, channels)

    if encoding in {"rgb8", "8uc3"}:
        rgb = arr
    elif encoding == "bgr8":
        rgb = arr[:, :, ::-1]
    elif encoding in {"rgba8", "8uc4"}:
        rgb = arr[:, :, :3]
    elif encoding == "bgra8":
        rgb = arr[:, :, [2, 1, 0]]
    else:
        rgb = np.repeat(arr, 3, axis=2)
    return np.ascontiguousarray(rgb, dtype=np.uint8)


def resize_image_if_needed(img: np.ndarray, height: int | None, width: int | None) -> np.ndarray:
    if height is None or width is None:
        return img
    if img.shape[0] == height and img.shape[1] == width:
        return img
    try:
        import cv2

        return cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
    except Exception as exc:
        raise RuntimeError(
            f"Image size is {img.shape[:2]} but requested {(height, width)}. "
            "Install opencv-python or set the camera height/width to match the bag."
        ) from exc


def state_vector_to_action_vector(state_vector: np.ndarray, state_names: list[str], action_names: list[str]) -> np.ndarray | None:
    state_index = {joint: i for i, joint in enumerate(state_names)}
    values: list[float] = []
    missing = [joint for joint in action_names if joint not in state_index]
    if missing:
        print(f"WARN: cannot map state -> action; action joints missing from state_names: {missing}")
        return None
    for joint in action_names:
        values.append(float(state_vector[state_index[joint]]))
    return np.asarray(values, dtype=np.float32)


def _load_string_map_args(
    *,
    default: dict[str, str] | None,
    file_arg: str | None,
    json_arg: str | None,
    assignment_args: list[str] | None,
    expected_name: str,
) -> dict[str, str]:
    return merge_string_maps(
        default or {},
        load_mapping(json_text=json_arg, file_path=file_arg, expected_name=expected_name),
        parse_assignment_list(assignment_args),
    )



def _load_topic_specs(*, json_arg: str | None, file_arg: str | None, expected_name: str) -> dict[str, dict[str, Any]]:
    specs: dict[str, dict[str, Any]] = {}
    def add_loaded(data: Any) -> None:
        if not data:
            return
        if isinstance(data, list):
            iterable = [(str(item.get("name")), item) for item in data if isinstance(item, dict)]
        elif isinstance(data, dict):
            iterable = data.items()
        else:
            raise ValueError(f"{expected_name} must be a mapping or list of specs")
        for name, value in iterable:
            if not isinstance(value, dict):
                raise ValueError(f"{expected_name}.{name} must be a mapping")
            spec = dict(value)
            spec.setdefault("name", str(name))
            spec.setdefault("feature_key", f"observation.{spec['name']}")
            specs[str(spec["name"])] = spec
    if file_arg:
        add_loaded(load_yaml_or_json_config(file_arg))
    if json_arg:
        add_loaded(json.loads(json_arg))
    return specs

def action_topic_arg_to_map(args: argparse.Namespace) -> dict[str, str]:
    controller_names = _load_string_map_args(
        default=DEFAULT_CONTROLLER_NAMES,
        file_arg=args.controller_names_file,
        json_arg=args.controller_names_json,
        assignment_args=args.controller_name,
        expected_name="controller_names",
    )
    topic_map = {group: f"/{controller}/joint_trajectory" for group, controller in controller_names.items()}

    # Backward-compatible individual topic flags. Apply them only when changed from defaults,
    # so --controller-name can still update the default-derived topic.
    legacy = {
        "rarm": args.rarm_action_topic,
        "rhand": args.rhand_action_topic,
        "larm": args.larm_action_topic,
        "lhand": args.lhand_action_topic,
        "waist": args.waist_action_topic,
        "lifter": args.lifter_action_topic,
        "head": args.head_action_topic,
    }
    for group, topic in legacy.items():
        if topic != DEFAULT_ACTION_TOPICS.get(group):
            topic_map[group] = topic

    explicit = _load_string_map_args(
        default=None,
        file_arg=args.action_topics_file,
        json_arg=args.action_topics_json,
        assignment_args=args.action_topic,
        expected_name="action_topics",
    )
    topic_map.update(explicit)
    return topic_map


def read_bag_data(
    bag_path: str | Path,
    *,
    storage_id: str | None,
    camera_specs: list[RosCameraSpec],
    joint_states_topic: str,
    action_topic_by_group: dict[str, str],
    extra_observation_specs: dict[str, dict[str, Any]],
    tf_observation_specs: dict[str, dict[str, Any]],
    joint_groups: dict[str, list[str]],
    state_joints: list[str],
    action_point_mode: str,
    strict_state_joints: bool,
) -> tuple[BagData, dict[str, str]]:
    reader, type_map = open_reader(bag_path, storage_id)

    camera_topic_by_name = {spec.name: spec.topic for spec in camera_specs}
    required = list(dict.fromkeys([*camera_topic_by_name.values(), joint_states_topic]))
    missing_required = [t for t in required if t not in type_map]
    if missing_required:
        raise RuntimeError(f"Missing required topics in bag: {missing_required}\nAvailable topics: {sorted(type_map)}")

    topics_to_read = list(required)
    topic_to_camera_specs: dict[str, list[RosCameraSpec]] = defaultdict(list)
    for spec in camera_specs:
        topic_to_camera_specs[spec.topic].append(spec)

    topic_to_extra_specs: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for spec in extra_observation_specs.values():
        topic = str(spec.get("topic", ""))
        if not topic:
            continue
        if topic in type_map:
            topics_to_read.append(topic)
            topic_to_extra_specs[topic].append(spec)
        else:
            print(f"WARN: extra observation topic not found in bag: {topic} ({spec.get('name')})")

    tf_topics: set[str] = set()
    for spec in tf_observation_specs.values():
        tf_topics.add(str(spec.get("dynamic_topic", "/tf")))
        tf_topics.add(str(spec.get("static_topic", "/tf_static")))
    for topic in sorted(tf_topics):
        if topic in type_map:
            topics_to_read.append(topic)
        else:
            print(f"WARN: TF topic not found in bag: {topic}")

    topic_to_group: dict[str, str] = {}
    for group, topic in action_topic_by_group.items():
        if not topic:
            continue
        if group not in joint_groups:
            print(f"WARN: action topic group {group!r} has no joint_groups entry; skipping topic {topic}")
            continue
        if topic in type_map:
            topics_to_read.append(topic)
            topic_to_group[topic] = group
        else:
            print(f"WARN: action topic for {group!r} not found in bag: {topic}")

    topics_to_read = list(dict.fromkeys(topics_to_read))
    msg_type_map = {topic: get_message(type_map[topic]) for topic in topics_to_read}

    images: dict[str, list[tuple[float, np.ndarray]]] = {spec.name: [] for spec in camera_specs}
    states: list[tuple[float, np.ndarray]] = []
    action_groups: dict[str, list[tuple[float, np.ndarray]]] = {g: [] for g in joint_groups}
    extra_observations: dict[str, list[tuple[float, np.ndarray]]] = {
        str(spec.get("feature_key", name)): [] for name, spec in {**extra_observation_specs, **tf_observation_specs}.items()
    }
    counts: dict[str, int] = defaultdict(int)

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in msg_type_map:
            continue
        msg = deserialize_message(data, msg_type_map[topic])
        t = bag_time_to_sec(t_ns)
        counts[topic] += 1

        if topic in topic_to_camera_specs:
            raw_img = image_msg_to_rgb_array(msg)
            for spec in topic_to_camera_specs[topic]:
                img = resize_image_if_needed(raw_img, spec.height, spec.width)
                images[spec.name].append((t, img.astype(np.uint8, copy=False)))

        elif topic == joint_states_topic:
            state = vector_from_joint_state(msg, state_joints, strict=strict_state_joints)
            if state is not None:
                states.append((t, state))

        elif topic in topic_to_extra_specs:
            for spec in topic_to_extra_specs[topic]:
                key = str(spec.get("feature_key", spec.get("name")))
                try:
                    extra_observations.setdefault(key, []).append((t, decode_numeric_message(msg, spec)))
                except Exception as exc:
                    print(f"WARN: failed to decode extra observation {key} from {topic}: {exc}")

        elif topic in tf_topics:
            for key, arr in decode_tf_message(msg, tf_observation_specs).items():
                extra_observations.setdefault(key, []).append((t, arr))

        elif topic in topic_to_group:
            group = topic_to_group[topic]
            action = vector_from_joint_trajectory(msg, joint_groups[group], point_mode=action_point_mode)
            if action is not None:
                action_groups[group].append((t, action))

    for series in images.values():
        series.sort(key=lambda x: x[0])
    states.sort(key=lambda x: x[0])
    for group in action_groups:
        action_groups[group].sort(key=lambda x: x[0])
    for key in extra_observations:
        extra_observations[key].sort(key=lambda x: x[0])

    return (
        BagData(
            images=images,
            states=states,
            action_groups=action_groups,
            extra_observations=extra_observations,
            counts=dict(counts),
            camera_topics=camera_topic_by_name,
            action_topic_by_group=action_topic_by_group,
        ),
        type_map,
    )


def value_range_array(series: list[tuple[float, np.ndarray]]) -> np.ndarray | None:
    if not series:
        return None
    arr = np.stack([v for _, v in series], axis=0)
    return arr.max(axis=0) - arr.min(axis=0)


def print_inspection(
    data: BagData,
    type_map: dict[str, str],
    state_joints: list[str],
    action_names: list[str],
    joint_groups: dict[str, list[str]],
) -> None:
    print("\n=== Topics in bag ===")
    for topic, typ in sorted(type_map.items()):
        print(f"{topic}: {typ}")

    print("\n=== Read counts ===")
    for k, v in sorted(data.counts.items()):
        print(f"{k}: {v}")
    for name, series in data.images.items():
        print(f"images parsed [{name} -> {data.camera_topics.get(name)}]: {len(series)}")
    print(f"states parsed: {len(data.states)}")
    for key, series in data.extra_observations.items():
        print(f"extra parsed [{key}]: {len(series)}")

    print(f"\n=== state layout: {len(state_joints)} axes ===")
    for i, name in enumerate(state_joints):
        print(f"state[{i:02d}]: {name}")

    print(f"\n=== action layout: {len(action_names)} axes ===")
    for i, name in enumerate(action_names):
        print(f"action[{i:02d}]: {name}")

    print("\n=== /joint_states selected joints ===")
    if not data.states:
        print("No selected state vectors parsed.")
    else:
        arr = np.stack([v for _, v in data.states], axis=0)
        for i, name in enumerate(state_joints):
            mn, mx = float(arr[:, i].min()), float(arr[:, i].max())
            print(f"{i:02d} {name}: min={mn:.6f}, max={mx:.6f}, range={mx - mn:.6f}")

    print("\n=== Candidate JointTrajectory action topics ===")
    for group, topic in data.action_topic_by_group.items():
        series = data.action_groups.get(group, [])
        status = "FOUND" if topic in type_map else "missing"
        print(f"{group:12s}: {topic} [{status}], parsed={len(series)}")
        ranges = value_range_array(series)
        if ranges is not None and group in joint_groups:
            for joint, rng in zip(joint_groups[group], ranges, strict=False):
                print(f"        {joint}: range={float(rng):.6f}")

    found_groups = [g for g, s in data.action_groups.items() if s]
    print("\nRecommended action extraction:")
    if found_groups:
        print(f"  Parsed trajectory groups: {found_groups}")
        print("  Use --action-mode auto to combine parsed command topics with future /joint_states fallback.")
    else:
        print("  No controller JointTrajectory topics were parsed. Use --action-mode future_joint_state.")

    for name, series in data.images.items():
        if series:
            print(f"\nFirst image shape [{name}]: {series[0][1].shape}, dtype={series[0][1].dtype}")


def action_from_future_state(
    data: BagData,
    t: float,
    *,
    state_names: list[str],
    action_names: list[str],
    action_future_sec: float,
    max_future_action_dt: float,
) -> np.ndarray | None:
    future_state = future_after(data.states, t, action_future_sec, max_dt=max_future_action_dt)
    if future_state is None:
        return None
    return state_vector_to_action_vector(np.asarray(future_state, dtype=np.float32), state_names, action_names)


def action_from_current_state(current_state: np.ndarray, *, state_names: list[str], action_names: list[str]) -> np.ndarray | None:
    return state_vector_to_action_vector(np.asarray(current_state, dtype=np.float32), state_names, action_names)


def action_from_trajectories_with_fallback(
    data: BagData,
    t: float,
    current_state: np.ndarray,
    *,
    state_names: list[str],
    action_names: list[str],
    joint_groups: dict[str, list[str]],
    action_future_sec: float,
    max_future_action_dt: float,
    max_trajectory_action_age: float,
    missing_action_fallback: str,
) -> np.ndarray | None:
    if missing_action_fallback == "future_joint_state":
        action = action_from_future_state(
            data,
            t,
            state_names=state_names,
            action_names=action_names,
            action_future_sec=action_future_sec,
            max_future_action_dt=max_future_action_dt,
        )
    elif missing_action_fallback == "current_joint_state":
        action = action_from_current_state(current_state, state_names=state_names, action_names=action_names)
    elif missing_action_fallback == "zero":
        action = np.zeros((len(action_names),), dtype=np.float32)
    elif missing_action_fallback == "error":
        action = np.zeros((len(action_names),), dtype=np.float32)
    else:
        raise ValueError(f"Unknown missing_action_fallback={missing_action_fallback!r}")

    if action is None:
        return None

    found_any = False
    for group, group_joints in joint_groups.items():
        series = data.action_groups.get(group, [])
        group_action = latest_before(series, t, max_age=max_trajectory_action_age)
        if group_action is None:
            if missing_action_fallback == "error" and data.action_topic_by_group.get(group):
                return None
            continue

        indices = [action_names.index(joint) for joint in group_joints if joint in action_names]
        src_values = [value for joint, value in zip(group_joints, group_action, strict=False) if joint in action_names]
        for dst_idx, value in zip(indices, src_values, strict=False):
            action[dst_idx] = float(value)
            found_any = True

    if missing_action_fallback == "error" and not found_any:
        return None
    return np.asarray(action, dtype=np.float32)


def build_action_vector(
    data: BagData,
    t: float,
    current_state: np.ndarray,
    *,
    action_mode: str,
    state_names: list[str],
    action_names: list[str],
    joint_groups: dict[str, list[str]],
    action_future_sec: float,
    max_future_action_dt: float,
    max_trajectory_action_age: float,
    missing_action_fallback: str,
) -> np.ndarray | None:
    if action_mode == "future_joint_state":
        return action_from_future_state(
            data,
            t,
            state_names=state_names,
            action_names=action_names,
            action_future_sec=action_future_sec,
            max_future_action_dt=max_future_action_dt,
        )
    if action_mode == "current_joint_state":
        return action_from_current_state(current_state, state_names=state_names, action_names=action_names)
    if action_mode in {"auto", "trajectory"}:
        fallback = missing_action_fallback
        if action_mode == "auto" and fallback == "error":
            fallback = "future_joint_state"
        return action_from_trajectories_with_fallback(
            data,
            t,
            current_state,
            state_names=state_names,
            action_names=action_names,
            joint_groups=joint_groups,
            action_future_sec=action_future_sec,
            max_future_action_dt=max_future_action_dt,
            max_trajectory_action_age=max_trajectory_action_age,
            missing_action_fallback=fallback,
        )
    raise ValueError(f"Unknown action_mode={action_mode!r}")


def build_synced_frames(
    data: BagData,
    *,
    camera_specs: list[RosCameraSpec],
    extra_observation_specs: dict[str, dict[str, Any]],
    tf_observation_specs: dict[str, dict[str, Any]],
    fps: float,
    action_mode: str,
    state_names: list[str],
    action_names: list[str],
    joint_groups: dict[str, list[str]],
    action_future_sec: float,
    max_image_dt: float,
    max_state_dt: float,
    max_extra_observation_dt: float,
    max_future_action_dt: float,
    max_trajectory_action_age: float,
    missing_action_fallback: str,
) -> list[dict[str, Any]]:
    for spec in camera_specs:
        if not data.images.get(spec.name):
            raise RuntimeError(f"No images parsed for camera {spec.name!r} topic {spec.topic!r}.")
    if not data.states:
        raise RuntimeError("No /joint_states selected vectors parsed.")

    starts = [data.states[0][0], *[data.images[spec.name][0][0] for spec in camera_specs]]
    ends = [data.states[-1][0], *[data.images[spec.name][-1][0] for spec in camera_specs]]

    uses_future = action_mode == "future_joint_state" or (
        action_mode in {"auto", "trajectory"} and missing_action_fallback == "future_joint_state"
    )
    if uses_future:
        ends.append(data.states[-1][0] - action_future_sec)

    start_t = max(starts)
    end_t = min(ends)
    if end_t <= start_t:
        raise RuntimeError(f"No overlapping time interval. start={start_t}, end={end_t}")

    dt = 1.0 / fps
    sample_times = np.arange(start_t, end_t, dt)
    frames: list[dict[str, Any]] = []

    for t in sample_times:
        state = nearest_by_time(data.states, t, max_dt=max_state_dt)
        if state is None:
            continue

        frame_images: dict[str, np.ndarray] = {}
        missing_image = False
        for spec in camera_specs:
            image = nearest_by_time(data.images[spec.name], t, max_dt=max_image_dt)
            if image is None:
                missing_image = True
                break
            frame_images[prefixed_image_key(spec.name)] = image
        if missing_image:
            continue

        frame_extras: dict[str, np.ndarray] = {}
        missing_extra = False
        for spec in list(extra_observation_specs.values()) + list(tf_observation_specs.values()):
            key = str(spec.get("feature_key", spec.get("name")))
            value = nearest_by_time(data.extra_observations.get(key, []), t, max_dt=max_extra_observation_dt)
            if value is None:
                if bool(spec.get("required", False)):
                    missing_extra = True
                    break
                value = np.zeros(shape_for_spec({**spec, "type": spec.get("type", "tf2_msgs/msg/TFMessage")}), dtype=np.float32)
            frame_extras[key] = np.asarray(value, dtype=np.float32)
        if missing_extra:
            continue

        action = build_action_vector(
            data,
            t,
            state,
            action_mode=action_mode,
            state_names=state_names,
            action_names=action_names,
            joint_groups=joint_groups,
            action_future_sec=action_future_sec,
            max_future_action_dt=max_future_action_dt,
            max_trajectory_action_age=max_trajectory_action_age,
            missing_action_fallback=missing_action_fallback,
        )
        if action is None:
            continue
        if action.shape != (len(action_names),):
            raise RuntimeError(f"Internal action shape mismatch: got {action.shape}, expected {(len(action_names),)}")

        frame: dict[str, Any] = dict(frame_images)
        frame.update(frame_extras)
        frame["observation.state"] = np.asarray(state, dtype=np.float32)
        frame["action"] = action.astype(np.float32)
        frames.append(frame)

    return frames


def create_lerobot_dataset(
    *,
    repo_id: str,
    root: str | None,
    fps: float,
    robot_type: str,
    camera_specs: list[RosCameraSpec],
    extra_observation_specs: dict[str, dict[str, Any]],
    tf_observation_specs: dict[str, dict[str, Any]],
    state_names: list[str],
    action_names: list[str],
    use_videos: bool,
    overwrite: bool,
    image_writer_processes: int,
    image_writer_threads: int,
):
    try:
        from lerobot.datasets import LeRobotDataset
    except Exception as exc:
        raise RuntimeError(
            "LeRobot is not importable. Install it in your ROS/Python environment first, e.g. `pip install lerobot` "
            "or your project's pinned LeRobot version."
        ) from exc

    if root is not None:
        target_dir = Path(root)
        if overwrite and target_dir.exists():
            shutil.rmtree(target_dir)

    image_dtype = "video" if use_videos else "image"
    features: dict[str, Any] = {}
    for spec in camera_specs:
        features[prefixed_image_key(spec.name)] = {
            "dtype": image_dtype,
            "shape": (spec.height, spec.width, 3),
            "names": ["height", "width", "channel"],
        }
    for spec in list(extra_observation_specs.values()) + list(tf_observation_specs.values()):
        key = str(spec.get("feature_key", spec.get("name")))
        features[key] = {
            "dtype": "float32",
            "shape": shape_for_spec({**spec, "type": spec.get("type", "tf2_msgs/msg/TFMessage")}),
            "names": names_for_spec({**spec, "type": spec.get("type", "tf2_msgs/msg/TFMessage")}),
        }
    features.update(
        {
            "observation.state": {
                "dtype": "float32",
                "shape": (len(state_names),),
                "names": state_names,
            },
            "action": {
                "dtype": "float32",
                "shape": (len(action_names),),
                "names": action_names,
            },
        }
    )
    fps_for_dataset = int(round(float(fps)))

    kwargs = {
        "repo_id": repo_id,
        "fps": fps_for_dataset,
        "robot_type": robot_type,
        "features": features,
        "root": root,
        "use_videos": use_videos,
        "image_writer_processes": image_writer_processes,
        "image_writer_threads": image_writer_threads,
    }

    sig = inspect.signature(LeRobotDataset.create)
    has_var_kwargs = any(p.kind == inspect.Parameter.VAR_KEYWORD for p in sig.parameters.values())
    filtered_kwargs = {k: v for k, v in kwargs.items() if v is not None and (has_var_kwargs or k in sig.parameters)}
    dataset = LeRobotDataset.create(**filtered_kwargs)

    if use_videos and hasattr(dataset, "start_image_writer"):
        try:
            image_writer = getattr(dataset, "image_writer", None)
            if not image_writer:
                dataset.start_image_writer()
        except Exception:
            pass

    return dataset


def add_episode_and_save(dataset: Any, frames: list[dict[str, Any]], task: str) -> None:
    for frame in frames:
        frame_with_task = dict(frame)
        frame_with_task["task"] = task
        dataset.add_frame(frame_with_task)
    try:
        dataset.save_episode(task=task)
    except TypeError:
        dataset.save_episode()


def finalize_dataset(dataset: Any) -> None:
    if hasattr(dataset, "finalize"):
        dataset.finalize()


def validate_names(state_joints: list[str], action_names: list[str]) -> None:
    if not state_joints:
        raise ValueError("state_joints is empty")
    if not action_names:
        raise ValueError("action_joints is empty")
    if len(set(state_joints)) != len(state_joints):
        duplicates = sorted({j for j in state_joints if state_joints.count(j) > 1})
        raise ValueError(f"Duplicate state joint names: {duplicates}")
    if len(set(action_names)) != len(action_names):
        duplicates = sorted({j for j in action_names if action_names.count(j) > 1})
        raise ValueError(f"Duplicate action joint names: {duplicates}")
    if state_joints != action_names:
        print("WARN: state_joints and action_joints differ. Future/current state fallback will map by joint name.")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    config_parser = argparse.ArgumentParser(add_help=False)
    config_parser.add_argument("--config-file", "--config", dest="config_file", default=None)
    cfg_args, _ = config_parser.parse_known_args(argv)
    config_defaults = rosbag_config_to_argparse_defaults(load_yaml_or_json_config(cfg_args.config_file))

    parser = argparse.ArgumentParser(
        description="Convert ROS 2 bag(s) to LeRobotDataset for noid mouse task",
        parents=[config_parser],
    )
    parser.add_argument("--bag", nargs="+", required="bag" not in config_defaults, help="One or more rosbag2 directories. Each bag becomes one episode.")
    parser.add_argument("--storage-id", default="auto", help="sqlite3, mcap, or auto")

    parser.add_argument("--repo-id", default="local/noid_mouse_pick_lift_place_configurable")
    parser.add_argument("--root", default=None, help="Optional LeRobot local root directory, e.g. ./lerobot_data")
    parser.add_argument("--robot-type", default="seed_noid")
    parser.add_argument("--task", default=DEFAULT_TASK)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--inspect-only", action="store_true")

    # Observation topics.
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC, help="Backward-compatible single camera topic.")
    parser.add_argument("--joint-states-topic", default=DEFAULT_JOINT_STATES_TOPIC)
    parser.add_argument("--image-height", type=int, default=480)
    parser.add_argument("--image-width", type=int, default=640)
    parser.add_argument(
        "--camera",
        action="append",
        default=[],
        help="Repeatable camera spec: name=/topic,height,width. Example: front=/camera/front/image_raw,480,640",
    )
    parser.add_argument("--cameras-json", default=None)
    parser.add_argument("--cameras-file", default=None)
    parser.add_argument("--extra-observation-topics-json", default=None, help="JSON mapping/list of typed numeric observation TopicSpecs.")
    parser.add_argument("--extra-observation-topics-file", default=None, help="JSON/YAML mapping/list of typed numeric observation TopicSpecs.")
    parser.add_argument("--tf-observations-json", default=None, help="JSON mapping/list of TF observation specs.")
    parser.add_argument("--tf-observations-file", default=None, help="JSON/YAML mapping/list of TF observation specs.")

    # Layout config.
    parser.add_argument("--joint-groups-json", default=None, help="Inline JSON mapping group -> joint list/string.")
    parser.add_argument("--joint-groups-file", default=None, help="JSON/YAML mapping group -> joint list/string.")
    parser.add_argument("--state-order-groups", default=",".join(DEFAULT_POLICY_GROUP_ORDER))
    parser.add_argument("--action-order-groups", default=",".join(DEFAULT_POLICY_GROUP_ORDER))
    parser.add_argument("--state-joints", default=None, help="Comma-separated observation.state joint order. Overrides --state-order-groups.")
    parser.add_argument("--action-joints", default=None, help="Comma-separated action joint order. Overrides --action-order-groups.")

    # Action topic config. Controller names are converted to /<controller>/joint_trajectory.
    parser.add_argument("--controller-name", action="append", default=[], help="Repeatable group=controller_name override.")
    parser.add_argument("--controller-names-json", default=None)
    parser.add_argument("--controller-names-file", default=None)
    parser.add_argument("--action-topic", action="append", default=[], help="Repeatable group=/topic override. Supports arbitrary topic count/group names.")
    parser.add_argument("--action-topics-json", default=None)
    parser.add_argument("--action-topics-file", default=None)

    # Legacy one-off topic flags kept for compatibility.
    parser.add_argument("--rarm-action-topic", default=DEFAULT_ACTION_TOPICS["rarm"])
    parser.add_argument("--rhand-action-topic", default=DEFAULT_ACTION_TOPICS["rhand"])
    parser.add_argument("--larm-action-topic", default=DEFAULT_ACTION_TOPICS["larm"])
    parser.add_argument("--lhand-action-topic", default=DEFAULT_ACTION_TOPICS["lhand"])
    parser.add_argument("--waist-action-topic", default=DEFAULT_ACTION_TOPICS["waist"])
    parser.add_argument("--lifter-action-topic", default=DEFAULT_ACTION_TOPICS["lifter"])
    parser.add_argument("--head-action-topic", default=DEFAULT_ACTION_TOPICS["head"])

    parser.add_argument("--action-point-mode", choices=["first", "final"], default="final")
    parser.add_argument(
        "--action-mode",
        choices=["auto", "trajectory", "future_joint_state", "current_joint_state"],
        default="auto",
        help="auto uses available controller JointTrajectory topics and fills missing groups from future /joint_states.",
    )
    parser.add_argument(
        "--missing-action-fallback",
        choices=["future_joint_state", "current_joint_state", "zero", "error"],
        default="future_joint_state",
        help="How auto/trajectory fills groups with no parsed command message near a frame.",
    )
    parser.add_argument("--action-future-sec", type=float, default=0.5)
    parser.add_argument("--max-future-action-dt", type=float, default=0.20)
    parser.add_argument("--max-trajectory-action-age", type=float, default=2.0)

    parser.add_argument("--max-image-dt", type=float, default=0.20)
    parser.add_argument("--max-state-dt", type=float, default=0.20)
    parser.add_argument("--max-extra-observation-dt", type=float, default=0.20)
    parser.add_argument("--no-strict-state-joints", action="store_true", help="Skip frames if selected state joints are missing instead of raising.")

    parser.add_argument("--no-videos", action="store_true", help="Use image storage instead of video storage if your LeRobot version supports it.")
    parser.add_argument("--image-writer-processes", type=int, default=2)
    parser.add_argument("--image-writer-threads", type=int, default=4)
    parser.add_argument("--debug-json", default=None, help="Optional path to save conversion summary JSON.")
    if config_defaults:
        parser.set_defaults(**config_defaults)
    return parser.parse_args(argv)


def main() -> None:
    args = parse_args()

    camera_specs = parse_camera_specs(
        camera_args=args.camera,
        cameras_json=args.cameras_json,
        cameras_file=args.cameras_file,
        default_topic=args.image_topic,
        default_height=args.image_height,
        default_width=args.image_width,
    )
    extra_observation_specs = _load_topic_specs(
        json_arg=args.extra_observation_topics_json,
        file_arg=args.extra_observation_topics_file,
        expected_name="extra_observation_topics",
    )
    tf_observation_specs = _load_topic_specs(
        json_arg=args.tf_observations_json,
        file_arg=args.tf_observations_file,
        expected_name="tf_observations",
    )
    joint_groups = merge_joint_groups(
        DEFAULT_JOINT_GROUPS,
        load_mapping(json_text=args.joint_groups_json, file_path=args.joint_groups_file, expected_name="joint_groups"),
    )

    state_joints = parse_csv_list(args.state_joints)
    if not state_joints:
        state_joints = joint_order_from_groups(parse_csv_list(args.state_order_groups), joint_groups)
    action_names = parse_csv_list(args.action_joints)
    if not action_names:
        action_names = joint_order_from_groups(parse_csv_list(args.action_order_groups), joint_groups)
    validate_names(state_joints, action_names)

    action_topic_by_group = action_topic_arg_to_map(args)

    all_episode_frames: list[list[dict[str, Any]]] = []
    summaries: list[dict[str, Any]] = []

    for bag in args.bag:
        print(f"\n### Reading bag: {bag}")
        data, type_map = read_bag_data(
            bag,
            storage_id=args.storage_id,
            camera_specs=camera_specs,
            joint_states_topic=args.joint_states_topic,
            action_topic_by_group=action_topic_by_group,
            extra_observation_specs=extra_observation_specs,
            tf_observation_specs=tf_observation_specs,
            joint_groups=joint_groups,
            state_joints=state_joints,
            action_point_mode=args.action_point_mode,
            strict_state_joints=not args.no_strict_state_joints,
        )

        if args.inspect_only:
            print_inspection(data, type_map, state_joints, action_names, joint_groups)
            continue

        frames = build_synced_frames(
            data,
            camera_specs=camera_specs,
            extra_observation_specs=extra_observation_specs,
            tf_observation_specs=tf_observation_specs,
            fps=args.fps,
            action_mode=args.action_mode,
            state_names=state_joints,
            action_names=action_names,
            joint_groups=joint_groups,
            action_future_sec=args.action_future_sec,
            max_image_dt=args.max_image_dt,
            max_state_dt=args.max_state_dt,
            max_extra_observation_dt=args.max_extra_observation_dt,
            max_future_action_dt=args.max_future_action_dt,
            max_trajectory_action_age=args.max_trajectory_action_age,
            missing_action_fallback=args.missing_action_fallback,
        )
        print(f"Built frames for episode: {len(frames)}")
        if not frames:
            raise RuntimeError(f"No synced frames generated for bag: {bag}")

        all_episode_frames.append(frames)
        summaries.append(
            {
                "bag": bag,
                "counts": data.counts,
                "parsed": {
                    "images": {name: len(series) for name, series in data.images.items()},
                    "states": len(data.states),
                    "frames": len(frames),
                    "action_groups": {group: len(series) for group, series in data.action_groups.items()},
                    "extra_observations": {key: len(series) for key, series in data.extra_observations.items()},
                },
                "camera_topics": data.camera_topics,
                "action_mode": args.action_mode,
                "missing_action_fallback": args.missing_action_fallback,
                "action_topic_by_group": action_topic_by_group,
                "state_joints": state_joints,
                "action_names": action_names,
            }
        )

    if args.inspect_only:
        return

    print("\n### Creating LeRobotDataset")
    dataset = create_lerobot_dataset(
        repo_id=args.repo_id,
        root=args.root,
        fps=args.fps,
        robot_type=args.robot_type,
        camera_specs=camera_specs,
        extra_observation_specs=extra_observation_specs,
        tf_observation_specs=tf_observation_specs,
        state_names=state_joints,
        action_names=action_names,
        use_videos=not args.no_videos,
        overwrite=args.overwrite,
        image_writer_processes=args.image_writer_processes,
        image_writer_threads=args.image_writer_threads,
    )

    for episode_idx, frames in enumerate(all_episode_frames):
        print(f"Saving episode {episode_idx}: {len(frames)} frames")
        add_episode_and_save(dataset, frames, args.task)

    print("Finalizing dataset")
    finalize_dataset(dataset)

    if args.debug_json:
        summary_path = Path(args.debug_json)
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        summary_path.write_text(json.dumps(summaries, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"Wrote debug summary: {summary_path}")

    print("\nDone.")
    print(f"repo_id: {args.repo_id}")
    if args.root:
        print(f"root: {args.root}")
    print("Load check example:")
    print("  from lerobot.datasets import LeRobotDataset")
    if args.root:
        print(f"  ds = LeRobotDataset('{args.repo_id}', root='{args.root}')")
    else:
        print(f"  ds = LeRobotDataset('{args.repo_id}')")
    cam_keys = ", ".join([repr(prefixed_image_key(spec.name)) for spec in camera_specs])
    print(f"  print(len(ds), ds[0]['observation.state'].shape, ds[0]['action'].shape, {cam_keys})")


if __name__ == "__main__":
    main()

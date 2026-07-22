from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable


@dataclass(frozen=True)
class RosCameraSpec:
    """A ROS image topic mapped to a LeRobot camera feature name."""

    name: str
    topic: str
    height: int
    width: int

    @property
    def shape(self) -> tuple[int, int, int]:
        return (self.height, self.width, 3)


def parse_csv_list(text: str | None) -> list[str]:
    if text is None:
        return []
    return [x.strip() for x in str(text).split(",") if x.strip()]


def _load_structured_file(path: str | Path) -> Any:
    path = Path(path).expanduser()
    text = path.read_text(encoding="utf-8")
    suffix = path.suffix.lower()
    if suffix in {".yaml", ".yml"}:
        try:
            import yaml  # type: ignore
        except Exception as exc:  # pragma: no cover - optional dependency
            raise RuntimeError(f"PyYAML is required to read YAML config files: {path}") from exc
        return yaml.safe_load(text)
    return json.loads(text)


def load_mapping(
    *,
    json_text: str | None = None,
    file_path: str | None = None,
    expected_name: str = "mapping",
) -> dict[str, Any]:
    """Load a dict from inline JSON and/or JSON/YAML file, with inline JSON taking precedence."""
    merged: dict[str, Any] = {}
    if file_path:
        data = _load_structured_file(file_path)
        if not isinstance(data, dict):
            raise ValueError(f"{expected_name} file must contain a mapping/dict: {file_path}")
        merged.update(data)
    if json_text:
        data = json.loads(json_text)
        if not isinstance(data, dict):
            raise ValueError(f"{expected_name} JSON must be an object/dict")
        merged.update(data)
    return merged


def parse_assignment_list(values: list[str] | None, *, value_parser: Callable[[str], Any] = str) -> dict[str, Any]:
    """Parse repeated KEY=VALUE CLI arguments."""
    out: dict[str, Any] = {}
    for raw in values or []:
        if "=" not in raw:
            raise ValueError(f"Expected KEY=VALUE, got: {raw!r}")
        key, value = raw.split("=", 1)
        key = key.strip()
        value = value.strip()
        if not key or not value:
            raise ValueError(f"Expected non-empty KEY=VALUE, got: {raw!r}")
        out[key] = value_parser(value)
    return out


def merge_string_maps(*maps: dict[str, Any] | None) -> dict[str, str]:
    out: dict[str, str] = {}
    for mapping in maps:
        if not mapping:
            continue
        for key, value in mapping.items():
            if value is None:
                continue
            out[str(key)] = str(value)
    return out


def merge_joint_groups(default_groups: dict[str, list[str]], *maps: dict[str, Any] | None) -> dict[str, list[str]]:
    out: dict[str, list[str]] = {str(k): [str(x) for x in v] for k, v in default_groups.items()}
    for mapping in maps:
        if not mapping:
            continue
        for key, value in mapping.items():
            if isinstance(value, str):
                joints = parse_csv_list(value)
            elif isinstance(value, (list, tuple)):
                joints = [str(x) for x in value]
            else:
                raise ValueError(f"joint group {key!r} must be a list or comma-separated string")
            if not joints:
                raise ValueError(f"joint group {key!r} is empty")
            out[str(key)] = joints
    return out


def joint_order_from_groups(group_names: list[str], joint_groups: dict[str, list[str]]) -> list[str]:
    joints: list[str] = []
    seen: set[str] = set()
    for group in group_names:
        if group not in joint_groups:
            raise KeyError(f"Unknown joint group {group!r}. Available groups: {sorted(joint_groups)}")
        for joint in joint_groups[group]:
            if joint not in seen:
                joints.append(joint)
                seen.add(joint)
    return joints


def parse_camera_spec(raw: str) -> RosCameraSpec:
    """Parse one camera spec.

    Accepted forms:
      camera1=/camera/color/image_raw,720,1280
      camera1:/camera/color/image_raw:720:1280
      {"name":"camera1","topic":"/camera/color/image_raw","height":720,"width":1280}
    """
    raw = raw.strip()
    if raw.startswith("{"):
        data = json.loads(raw)
        return camera_spec_from_mapping(data)

    if "=" in raw:
        name, rest = raw.split("=", 1)
        parts = [p.strip() for p in rest.split(",")]
    else:
        parts_all = [p.strip() for p in raw.split(":")]
        if len(parts_all) < 4:
            raise ValueError(f"Camera spec must be name=topic,height,width or name:topic:height:width, got: {raw!r}")
        name = parts_all[0]
        parts = [":".join(parts_all[1:-2]), parts_all[-2], parts_all[-1]]

    if len(parts) != 3:
        raise ValueError(f"Camera spec must contain topic,height,width, got: {raw!r}")
    topic, height, width = parts
    name = name.strip()
    if not name or not topic:
        raise ValueError(f"Camera name/topic cannot be empty: {raw!r}")
    return RosCameraSpec(name=name, topic=topic, height=int(height), width=int(width))


def camera_spec_from_mapping(data: dict[str, Any], *, name: str | None = None) -> RosCameraSpec:
    if not isinstance(data, dict):
        raise ValueError(f"Camera spec must be a dict, got {type(data).__name__}")
    cam_name = str(name if name is not None else data.get("name", "")).strip()
    topic = str(data.get("topic", data.get("image_topic", ""))).strip()
    height = data.get("height", data.get("image_height"))
    width = data.get("width", data.get("image_width"))
    if not cam_name or not topic or height is None or width is None:
        raise ValueError(f"Camera spec requires name, topic, height, width: {data}")
    return RosCameraSpec(name=cam_name, topic=topic, height=int(height), width=int(width))


def parse_camera_specs(
    *,
    camera_args: list[str] | None,
    cameras_json: str | None,
    cameras_file: str | None,
    default_topic: str,
    default_height: int,
    default_width: int,
    default_name: str = "camera1",
) -> list[RosCameraSpec]:
    specs: dict[str, RosCameraSpec] = {}

    loaded = load_mapping(json_text=cameras_json, file_path=cameras_file, expected_name="cameras")
    for key, value in loaded.items():
        if isinstance(value, str):
            spec = parse_camera_spec(f"{key}={value}")
        elif isinstance(value, dict):
            spec = camera_spec_from_mapping(value, name=str(key))
        else:
            raise ValueError(f"Camera entry {key!r} must be a string or dict")
        specs[spec.name] = spec

    for raw in camera_args or []:
        spec = parse_camera_spec(raw)
        specs[spec.name] = spec

    if not specs:
        specs[default_name] = RosCameraSpec(default_name, default_topic, int(default_height), int(default_width))

    return list(specs.values())


def camera_topics(specs: list[RosCameraSpec]) -> dict[str, str]:
    return {spec.name: spec.topic for spec in specs}


def camera_shapes(specs: list[RosCameraSpec]) -> dict[str, tuple[int, int, int]]:
    return {spec.name: spec.shape for spec in specs}


def prefixed_image_key(camera_name: str) -> str:
    return f"observation.images.{camera_name}"


def _get_path(data: dict[str, Any], *keys: str) -> Any:
    cur: Any = data
    for key in keys:
        if not isinstance(cur, dict) or key not in cur:
            return None
        cur = cur[key]
    return cur


def _set_if_present(out: dict[str, Any], dest: str, data: dict[str, Any], *keys: str, transform: Callable[[Any], Any] | None = None) -> None:
    value = _get_path(data, *keys)
    if value is None:
        return
    out[dest] = transform(value) if transform else value


def _to_csv(value: Any) -> str:
    if isinstance(value, str):
        return value
    if isinstance(value, (list, tuple)):
        return ",".join(str(x) for x in value)
    raise ValueError(f"Expected string/list for CSV value, got {type(value).__name__}")


def _to_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False)


def load_yaml_or_json_config(path: str | Path | None) -> dict[str, Any]:
    if not path:
        return {}
    data = _load_structured_file(path)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"config file must contain a mapping/dict: {path}")
    return data



def _normalize_msg_type(value: str | None) -> str:
    try:
        from .topic_decoders import normalize_msg_type
    except Exception:  # pragma: no cover - direct script/importlib execution fallback
        import importlib.util
        import sys
        module_path = Path(__file__).with_name("topic_decoders.py")
        spec = importlib.util.spec_from_file_location("topic_decoders", module_path)
        if spec is None or spec.loader is None:
            raise
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        normalize_msg_type = module.normalize_msg_type
    return normalize_msg_type(value)


def _normalize_topic_spec(name: str, data: Any, *, default_topic: str | None = None, default_type: str | None = None, default_feature_prefix: str = "observation") -> dict[str, Any]:
    if isinstance(data, str):
        data = {"topic": data}
    if not isinstance(data, dict):
        raise ValueError(f"Topic spec {name!r} must be a mapping or string, got {type(data).__name__}")
    spec = dict(data)
    spec.setdefault("name", name)
    if default_topic is not None:
        spec.setdefault("topic", default_topic)
    if default_type is not None:
        spec.setdefault("type", default_type)
    if "msg_type" in spec and "type" not in spec:
        spec["type"] = spec["msg_type"]
    spec["type"] = _normalize_msg_type(str(spec.get("type", "")))
    if "feature_key" not in spec:
        if spec["type"] == "sensor_msgs/msg/Image":
            spec["feature_key"] = f"observation.images.{name}"
        elif spec["type"] == "tf2_msgs/msg/TFMessage":
            spec["feature_key"] = f"observation.transforms.{name}"
        else:
            spec["feature_key"] = f"{default_feature_prefix}.{name}"
    spec["use_as_policy_input"] = bool(spec.get("use_as_policy_input", False))
    return spec


def parse_extra_observation_specs(config: dict[str, Any]) -> list[dict[str, Any]]:
    """Return typed non-image/non-joint observation topic specs from config.

    These are generic numeric observation topics such as IMU, WrenchStamped,
    PoseStamped, Odometry, and TwistStamped. Each spec should contain type,
    topic, feature_key, and optionally fields/names/shape/use_as_policy_input.
    """
    obs = _get_path(config, "topics", "observation") or {}
    if not isinstance(obs, dict):
        return []
    raw = obs.get("extra", obs.get("extra_observations", {})) or {}
    if not isinstance(raw, dict):
        raise ValueError("topics.observation.extra must be a mapping")
    specs: list[dict[str, Any]] = []
    for name, value in raw.items():
        spec = _normalize_topic_spec(str(name), value, default_feature_prefix="observation")
        if spec["type"] in {"sensor_msgs/msg/Image", "sensor_msgs/msg/JointState", "tf2_msgs/msg/TFMessage"}:
            raise ValueError(f"Extra observation {name!r} has reserved type {spec['type']}; use cameras/joint_state/transforms sections")
        if not spec.get("topic"):
            raise ValueError(f"Extra observation {name!r} requires topic")
        specs.append(spec)
    return specs


def parse_tf_observation_specs(config: dict[str, Any]) -> list[dict[str, Any]]:
    obs = _get_path(config, "topics", "observation") or {}
    if not isinstance(obs, dict):
        return []
    raw = obs.get("transforms", obs.get("tf", {})) or {}
    if not isinstance(raw, dict):
        raise ValueError("topics.observation.transforms must be a mapping")
    specs: list[dict[str, Any]] = []
    default_dynamic = obs.get("tf_topic", "/tf")
    default_static = obs.get("tf_static_topic", "/tf_static")
    for name, value in raw.items():
        spec = _normalize_topic_spec(str(name), value, default_type="tf2_msgs/msg/TFMessage", default_feature_prefix="observation.transforms")
        spec.setdefault("dynamic_topic", spec.get("topic", default_dynamic))
        spec.setdefault("static_topic", default_static)
        spec.setdefault("feature_key", f"observation.transforms.{name}")
        if not spec.get("parent_frame") or not spec.get("child_frame"):
            raise ValueError(f"TF observation {name!r} requires parent_frame and child_frame")
        specs.append(spec)
    return specs


def observation_specs_json(config: dict[str, Any]) -> str | None:
    specs = parse_extra_observation_specs(config)
    return _to_json(specs) if specs else None


def tf_specs_json(config: dict[str, Any]) -> str | None:
    specs = parse_tf_observation_specs(config)
    return _to_json(specs) if specs else None

def bridge_config_to_argparse_defaults(config: dict[str, Any]) -> dict[str, Any]:
    """Convert the model bridge YAML structure to argparse defaults.

    This supports both the legacy simple layout and the typed TopicSpec layout:
      topics.observation.joint_states: /joint_states
      topics.observation.joint_states: {topic: /joint_states, type: sensor_msgs/msg/JointState}
      topics.observation.cameras.<name>: {topic, type: sensor_msgs/msg/Image, height, width, ...}
      topics.observation.extra.<name>: {topic, type, feature_key, ...}
      topics.observation.transforms.<name>: {dynamic_topic, static_topic, parent_frame, child_frame, ...}
    """
    out: dict[str, Any] = {}

    for key in [
        "policy_path", "policy_type", "device", "task", "dataset_repo_id", "dataset_root",
        "rate_hz", "print_every", "max_steps",
    ]:
        _set_if_present(out, key, config, "runtime", key)

    for key in ["robot_id", "node_name"]:
        _set_if_present(out, key, config, "node", key)

    joint_states = _get_path(config, "topics", "observation", "joint_states")
    if isinstance(joint_states, dict):
        if joint_states.get("topic") is not None:
            out["joint_states_topic"] = str(joint_states["topic"])
    else:
        _set_if_present(out, "joint_states_topic", config, "topics", "observation", "joint_states")

    cameras = _get_path(config, "topics", "observation", "cameras")
    if cameras is not None:
        out["cameras_json"] = _to_json(cameras)
        if isinstance(cameras, dict) and len(cameras) == 1:
            first = next(iter(cameras.values()))
            if isinstance(first, dict):
                if first.get("topic") is not None:
                    out["image_topic"] = str(first["topic"])
                if first.get("height") is not None:
                    out["image_height"] = int(first["height"])
                if first.get("width") is not None:
                    out["image_width"] = int(first["width"])

    extra_json = observation_specs_json(config)
    if extra_json:
        out["extra_observation_topics_json"] = extra_json
    tf_json = tf_specs_json(config)
    if tf_json:
        out["tf_observations_json"] = tf_json

    action_topics = _get_path(config, "topics", "action") or {}
    if isinstance(action_topics, dict):
        if action_topics.get("base_twist") is not None:
            value = action_topics["base_twist"]
            out["base_twist_topic"] = str(value.get("topic") if isinstance(value, dict) else value)
        if action_topics.get("hand_service") is not None:
            value = action_topics["hand_service"]
            out["hand_service_name"] = str(value.get("name", value.get("topic")) if isinstance(value, dict) else value)
        if action_topics.get("joint_command_transport") is not None:
            out["joint_command_transport"] = str(action_topics["joint_command_transport"])
        if action_topics.get("controller_names") is not None:
            out["controller_names_json"] = _to_json(action_topics["controller_names"])
        if action_topics.get("joint_trajectory") is not None:
            jt = action_topics["joint_trajectory"]
            if isinstance(jt, dict):
                jt = {k: (v.get("topic") if isinstance(v, dict) else v) for k, v in jt.items()}
            out["joint_trajectory_topics_json"] = _to_json(jt)
        if action_topics.get("follow_joint_trajectory") is not None:
            ft = action_topics["follow_joint_trajectory"]
            if isinstance(ft, dict):
                ft = {k: (v.get("topic", v.get("action")) if isinstance(v, dict) else v) for k, v in ft.items()}
            out["follow_joint_trajectory_actions_json"] = _to_json(ft)

    layout = config.get("layout", {})
    if isinstance(layout, dict):
        if layout.get("state_order_groups") is not None:
            out["state_order_groups"] = _to_csv(layout["state_order_groups"])
        if layout.get("action_order_groups") is not None:
            out["action_order_groups"] = _to_csv(layout["action_order_groups"])
        if layout.get("state_joints") is not None:
            out["state_joints"] = _to_csv(layout["state_joints"])
        if layout.get("action_joints") is not None:
            out["action_joints"] = _to_csv(layout["action_joints"])
        if layout.get("joint_groups") is not None:
            out["joint_groups_json"] = _to_json(layout["joint_groups"])

    for key in [
        "trajectory_duration", "max_delta_rad", "max_thumb_delta_rad",
        "action_smoothing_alpha", "max_policy_action_delta",
        "wait_for_action_servers", "wait_for_joint_state", "stale_joint_state", "stale_image",
        "stale_extra_observation", "initial_observation_timeout", "initial_observation_retry_sec",
        "hand_open_threshold", "hand_close_threshold", "hand_service_timeout",
    ]:
        _set_if_present(out, key, config, "safety", key)

    command = config.get("command", {})
    if isinstance(command, dict):
        for key in [
            "enable_command", "enable_rhand_trajectory", "enable_larm_trajectory",
            "enable_lhand_trajectory", "enable_waist_trajectory", "enable_lifter_trajectory",
            "enable_head_trajectory", "enable_all_trajectories", "enable_hand_service",
        ]:
            if key in command:
                out[key] = bool(command[key])

    async_cfg = config.get("async_policy", {})
    if isinstance(async_cfg, dict):
        key_map = {
            "enabled": "async_policy_prefetch",
            "buffer_size": "async_buffer_size",
            "prefill_actions": "async_prefill_actions",
            "prefill_timeout": "async_prefill_timeout",
            "action_timeout": "async_action_timeout",
            "reuse_last_action_on_underrun": "reuse_last_action_on_underrun",
        }
        for src, dst in key_map.items():
            if src in async_cfg:
                out[dst] = bool(async_cfg[src]) if src in {"enabled", "reuse_last_action_on_underrun"} else async_cfg[src]

    return out


def rosbag_config_to_argparse_defaults(config: dict[str, Any]) -> dict[str, Any]:
    """Convert the rosbag converter YAML structure to argparse defaults."""
    out: dict[str, Any] = {}
    dataset = config.get("dataset", {})
    if isinstance(dataset, dict):
        for key in ["bag", "storage_id", "repo_id", "root", "robot_type", "task", "fps", "overwrite", "inspect_only"]:
            if key in dataset:
                out[key] = dataset[key]

    joint_states = _get_path(config, "topics", "observation", "joint_states")
    if isinstance(joint_states, dict):
        if joint_states.get("topic") is not None:
            out["joint_states_topic"] = str(joint_states["topic"])
    else:
        _set_if_present(out, "joint_states_topic", config, "topics", "observation", "joint_states")

    cameras = _get_path(config, "topics", "observation", "cameras")
    if cameras is not None:
        out["cameras_json"] = _to_json(cameras)
        if isinstance(cameras, dict) and len(cameras) == 1:
            first = next(iter(cameras.values()))
            if isinstance(first, dict):
                if first.get("topic") is not None:
                    out["image_topic"] = str(first["topic"])
                if first.get("height") is not None:
                    out["image_height"] = int(first["height"])
                if first.get("width") is not None:
                    out["image_width"] = int(first["width"])

    extra_json = observation_specs_json(config)
    if extra_json:
        out["extra_observation_topics_json"] = extra_json
    tf_json = tf_specs_json(config)
    if tf_json:
        out["tf_observations_json"] = tf_json

    action_topics = _get_path(config, "topics", "action") or {}
    if isinstance(action_topics, dict):
        if action_topics.get("controller_names") is not None:
            out["controller_names_json"] = _to_json(action_topics["controller_names"])
        if action_topics.get("joint_trajectory") is not None:
            jt = action_topics["joint_trajectory"]
            if isinstance(jt, dict):
                jt = {k: (v.get("topic") if isinstance(v, dict) else v) for k, v in jt.items()}
            out["action_topics_json"] = _to_json(jt)

    layout = config.get("layout", {})
    if isinstance(layout, dict):
        if layout.get("state_order_groups") is not None:
            out["state_order_groups"] = _to_csv(layout["state_order_groups"])
        if layout.get("action_order_groups") is not None:
            out["action_order_groups"] = _to_csv(layout["action_order_groups"])
        if layout.get("state_joints") is not None:
            out["state_joints"] = _to_csv(layout["state_joints"])
        if layout.get("action_joints") is not None:
            out["action_joints"] = _to_csv(layout["action_joints"])
        if layout.get("joint_groups") is not None:
            out["joint_groups_json"] = _to_json(layout["joint_groups"])

    action = config.get("action", {})
    if isinstance(action, dict):
        for key in [
            "action_point_mode", "action_mode", "missing_action_fallback", "action_future_sec",
            "max_future_action_dt", "max_trajectory_action_age",
        ]:
            if key in action:
                out[key] = action[key]

    sync = config.get("sync", {})
    if isinstance(sync, dict):
        for key in ["max_image_dt", "max_state_dt", "max_extra_observation_dt", "no_strict_state_joints"]:
            if key in sync:
                out[key] = sync[key]

    writer = config.get("writer", {})
    if isinstance(writer, dict):
        for key in ["no_videos", "image_writer_processes", "image_writer_threads", "debug_json"]:
            if key in writer:
                out[key] = writer[key]

    return out

from __future__ import annotations

import math
from typing import Any

import numpy as np


def normalize_msg_type(msg_type: str | None) -> str:
    value = (msg_type or "").strip()
    aliases = {
        "image": "sensor_msgs/msg/Image",
        "sensor_msgs/Image": "sensor_msgs/msg/Image",
        "sensor_msgs/msg/Image": "sensor_msgs/msg/Image",
        "joint_state": "sensor_msgs/msg/JointState",
        "joint_states": "sensor_msgs/msg/JointState",
        "sensor_msgs/JointState": "sensor_msgs/msg/JointState",
        "sensor_msgs/msg/JointState": "sensor_msgs/msg/JointState",
        "joint_trajectory": "trajectory_msgs/msg/JointTrajectory",
        "trajectory_msgs/JointTrajectory": "trajectory_msgs/msg/JointTrajectory",
        "trajectory_msgs/msg/JointTrajectory": "trajectory_msgs/msg/JointTrajectory",
        "tf": "tf2_msgs/msg/TFMessage",
        "tf_message": "tf2_msgs/msg/TFMessage",
        "tf2_msgs/TFMessage": "tf2_msgs/msg/TFMessage",
        "tf2_msgs/msg/TFMessage": "tf2_msgs/msg/TFMessage",
        "imu": "sensor_msgs/msg/Imu",
        "sensor_msgs/Imu": "sensor_msgs/msg/Imu",
        "sensor_msgs/msg/Imu": "sensor_msgs/msg/Imu",
        "wrench": "geometry_msgs/msg/Wrench",
        "geometry_msgs/Wrench": "geometry_msgs/msg/Wrench",
        "geometry_msgs/msg/Wrench": "geometry_msgs/msg/Wrench",
        "wrench_stamped": "geometry_msgs/msg/WrenchStamped",
        "geometry_msgs/WrenchStamped": "geometry_msgs/msg/WrenchStamped",
        "geometry_msgs/msg/WrenchStamped": "geometry_msgs/msg/WrenchStamped",
        "pose": "geometry_msgs/msg/Pose",
        "geometry_msgs/Pose": "geometry_msgs/msg/Pose",
        "geometry_msgs/msg/Pose": "geometry_msgs/msg/Pose",
        "pose_stamped": "geometry_msgs/msg/PoseStamped",
        "geometry_msgs/PoseStamped": "geometry_msgs/msg/PoseStamped",
        "geometry_msgs/msg/PoseStamped": "geometry_msgs/msg/PoseStamped",
        "twist": "geometry_msgs/msg/Twist",
        "geometry_msgs/Twist": "geometry_msgs/msg/Twist",
        "geometry_msgs/msg/Twist": "geometry_msgs/msg/Twist",
        "twist_stamped": "geometry_msgs/msg/TwistStamped",
        "geometry_msgs/TwistStamped": "geometry_msgs/msg/TwistStamped",
        "geometry_msgs/msg/TwistStamped": "geometry_msgs/msg/TwistStamped",
        "transform_stamped": "geometry_msgs/msg/TransformStamped",
        "geometry_msgs/TransformStamped": "geometry_msgs/msg/TransformStamped",
        "geometry_msgs/msg/TransformStamped": "geometry_msgs/msg/TransformStamped",
        "odometry": "nav_msgs/msg/Odometry",
        "odom": "nav_msgs/msg/Odometry",
        "nav_msgs/Odometry": "nav_msgs/msg/Odometry",
        "nav_msgs/msg/Odometry": "nav_msgs/msg/Odometry",
    }
    return aliases.get(value, value)


def _vec3(obj: Any, prefix: str) -> tuple[list[float], list[str]]:
    return [float(obj.x), float(obj.y), float(obj.z)], [f"{prefix}.x", f"{prefix}.y", f"{prefix}.z"]


def _quat(obj: Any, prefix: str) -> tuple[list[float], list[str]]:
    return [float(obj.x), float(obj.y), float(obj.z), float(obj.w)], [f"{prefix}.x", f"{prefix}.y", f"{prefix}.z", f"{prefix}.w"]


def _rpy_from_quat(q: Any) -> list[float]:
    # ROS quaternions are x, y, z, w.
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [roll, pitch, yaw]


def _pose_vector(pose: Any, *, prefix: str = "pose", representation: str = "xyz_quat") -> tuple[list[float], list[str]]:
    vals, names = _vec3(pose.position, f"{prefix}.position")
    if representation == "xyz_rpy":
        vals += _rpy_from_quat(pose.orientation)
        names += [f"{prefix}.orientation.roll", f"{prefix}.orientation.pitch", f"{prefix}.orientation.yaw"]
    else:
        qv, qn = _quat(pose.orientation, f"{prefix}.orientation")
        vals += qv
        names += qn
    return vals, names


def _twist_vector(twist: Any, *, prefix: str = "twist") -> tuple[list[float], list[str]]:
    lv, ln = _vec3(twist.linear, f"{prefix}.linear")
    av, an = _vec3(twist.angular, f"{prefix}.angular")
    return lv + av, ln + an


def _wrench_vector(wrench: Any, *, prefix: str = "wrench") -> tuple[list[float], list[str]]:
    fv, fn = _vec3(wrench.force, f"{prefix}.force")
    tv, tn = _vec3(wrench.torque, f"{prefix}.torque")
    return fv + tv, fn + tn


def numeric_default_names(msg_type: str, *, representation: str = "xyz_quat", fields: list[str] | None = None) -> list[str]:
    msg_type = normalize_msg_type(msg_type)
    fields = list(fields or [])
    if msg_type == "sensor_msgs/msg/Imu":
        selected = fields or ["orientation", "angular_velocity", "linear_acceleration"]
        names: list[str] = []
        if "orientation" in selected:
            if representation == "xyz_rpy":
                names += ["orientation.roll", "orientation.pitch", "orientation.yaw"]
            else:
                names += ["orientation.x", "orientation.y", "orientation.z", "orientation.w"]
        if "angular_velocity" in selected:
            names += ["angular_velocity.x", "angular_velocity.y", "angular_velocity.z"]
        if "linear_acceleration" in selected:
            names += ["linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"]
        return names
    if msg_type in {"geometry_msgs/msg/Wrench", "geometry_msgs/msg/WrenchStamped"}:
        return ["force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"]
    if msg_type in {"geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped"}:
        return ["linear.x", "linear.y", "linear.z", "angular.x", "angular.y", "angular.z"]
    if msg_type in {"geometry_msgs/msg/Pose", "geometry_msgs/msg/PoseStamped", "geometry_msgs/msg/TransformStamped", "tf2_msgs/msg/TFMessage"}:
        return ["x", "y", "z", "roll", "pitch", "yaw"] if representation == "xyz_rpy" else ["x", "y", "z", "qx", "qy", "qz", "qw"]
    if msg_type == "nav_msgs/msg/Odometry":
        pose_names = [f"pose.{n}" for n in (["x", "y", "z", "roll", "pitch", "yaw"] if representation == "xyz_rpy" else ["x", "y", "z", "qx", "qy", "qz", "qw"])]
        return pose_names + ["twist.linear.x", "twist.linear.y", "twist.linear.z", "twist.angular.x", "twist.angular.y", "twist.angular.z"]
    raise ValueError(f"No default numeric names for msg_type={msg_type!r}")


def decode_numeric_message(msg: Any, spec: dict[str, Any]) -> np.ndarray:
    msg_type = normalize_msg_type(str(spec.get("type", spec.get("msg_type", ""))))
    representation = str(spec.get("representation", "xyz_quat"))
    fields = list(spec.get("fields") or [])

    values: list[float] = []
    if msg_type == "sensor_msgs/msg/Imu":
        selected = fields or ["orientation", "angular_velocity", "linear_acceleration"]
        if "orientation" in selected:
            if representation == "xyz_rpy":
                values += _rpy_from_quat(msg.orientation)
            else:
                qv, _ = _quat(msg.orientation, "orientation")
                values += qv
        if "angular_velocity" in selected:
            v, _ = _vec3(msg.angular_velocity, "angular_velocity")
            values += v
        if "linear_acceleration" in selected:
            v, _ = _vec3(msg.linear_acceleration, "linear_acceleration")
            values += v
    elif msg_type == "geometry_msgs/msg/WrenchStamped":
        values, _ = _wrench_vector(msg.wrench, prefix="wrench")
    elif msg_type == "geometry_msgs/msg/Wrench":
        values, _ = _wrench_vector(msg, prefix="wrench")
    elif msg_type == "geometry_msgs/msg/PoseStamped":
        values, _ = _pose_vector(msg.pose, prefix="pose", representation=representation)
    elif msg_type == "geometry_msgs/msg/Pose":
        values, _ = _pose_vector(msg, prefix="pose", representation=representation)
    elif msg_type == "geometry_msgs/msg/TwistStamped":
        values, _ = _twist_vector(msg.twist, prefix="twist")
    elif msg_type == "geometry_msgs/msg/Twist":
        values, _ = _twist_vector(msg, prefix="twist")
    elif msg_type == "geometry_msgs/msg/TransformStamped":
        values = transform_stamped_to_vector(msg, representation=representation).tolist()
    elif msg_type == "nav_msgs/msg/Odometry":
        pv, _ = _pose_vector(msg.pose.pose, prefix="pose", representation=representation)
        tv, _ = _twist_vector(msg.twist.twist, prefix="twist")
        values = pv + tv
    else:
        raise ValueError(f"Unsupported numeric observation msg_type={msg_type!r}")

    arr = np.asarray(values, dtype=np.float32)
    expected_shape = spec.get("shape")
    if expected_shape is not None:
        expected_len = int(expected_shape[0] if isinstance(expected_shape, (list, tuple)) else expected_shape)
        if arr.shape != (expected_len,):
            raise ValueError(f"Decoded {spec.get('name')} shape {arr.shape}, expected {(expected_len,)}")
    return arr


def transform_stamped_to_vector(msg: Any, *, representation: str = "xyz_quat") -> np.ndarray:
    t = msg.transform.translation
    r = msg.transform.rotation
    values = [float(t.x), float(t.y), float(t.z)]
    if representation == "xyz_rpy":
        values += _rpy_from_quat(r)
    else:
        values += [float(r.x), float(r.y), float(r.z), float(r.w)]
    return np.asarray(values, dtype=np.float32)


def decode_tf_message(msg: Any, tf_specs: dict[str, dict[str, Any]]) -> dict[str, np.ndarray]:
    out: dict[str, np.ndarray] = {}
    for tr in getattr(msg, "transforms", []):
        parent = str(getattr(tr.header, "frame_id", ""))
        child = str(getattr(tr, "child_frame_id", ""))
        for name, spec in tf_specs.items():
            if parent == str(spec.get("parent_frame")) and child == str(spec.get("child_frame")):
                feature_key = str(spec.get("feature_key", f"observation.transforms.{name}"))
                out[feature_key] = transform_stamped_to_vector(tr, representation=str(spec.get("representation", "xyz_quat")))
    return out


def names_for_spec(spec: dict[str, Any]) -> list[str]:
    if spec.get("names") is not None:
        return [str(x) for x in spec["names"]]
    msg_type = normalize_msg_type(str(spec.get("type", spec.get("msg_type", ""))))
    return numeric_default_names(msg_type, representation=str(spec.get("representation", "xyz_quat")), fields=spec.get("fields"))


def shape_for_spec(spec: dict[str, Any]) -> tuple[int, ...]:
    if spec.get("shape") is not None:
        raw = spec["shape"]
        if isinstance(raw, int):
            return (int(raw),)
        return tuple(int(x) for x in raw)
    return (len(names_for_spec(spec)),)

from __future__ import annotations

import math
import threading
import time
from io import BytesIO
from dataclasses import dataclass
from typing import Any

import numpy as np

try:
    from lerobot.cameras import make_cameras_from_configs
except ImportError:  # pragma: no cover
    from lerobot.cameras.utils import make_cameras_from_configs

try:
    from lerobot.robots.robot import Robot
except ImportError:  # pragma: no cover
    from lerobot.robots import Robot

from lerobot.types import RobotAction, RobotObservation

from .config_seed_noid import SeedNoidConfig


@dataclass(frozen=True)
class _RosDeps:
    rclpy: Any
    JointState: Any
    Image: Any
    CompressedImage: Any
    Twist: Any
    JointTrajectory: Any
    JointTrajectoryPoint: Any
    Duration: Any
    FollowJointTrajectory: Any
    ActionClient: Any
    MultiThreadedExecutor: Any
    qos_profile_sensor_data: Any
    RunScript: Any | None
    ScriptReqJNoInterf: Any | None


def _load_ros_deps(*, require_hand_service: bool) -> _RosDeps:
    """Import ROS 2 lazily so LeRobot can discover the plugin before ROS is sourced."""
    try:
        import rclpy
        from builtin_interfaces.msg import Duration
        from control_msgs.action import FollowJointTrajectory
        from geometry_msgs.msg import Twist
        from rclpy.action import ActionClient
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, CompressedImage, JointState
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    except Exception as exc:  # pragma: no cover - requires ROS 2 Jazzy environment
        raise RuntimeError(
            "ROS 2 Python packages are not importable. Source ROS 2 and the robot workspace first:\n"
            "  source /opt/ros/jazzy/setup.bash\n"
            "  source ~<robot_workspace>/install/setup.bash\n"
            "Use a virtual environment created with --system-site-packages."
        ) from exc

    run_script = None
    script_request = None
    if require_hand_service:
        try:
            from aero_controller_msgs.msg import ScriptReqJNoInterf
            from aero_controller_msgs.srv import RunScript

            run_script = RunScript
            script_request = ScriptReqJNoInterf
        except Exception as exc:  # pragma: no cover
            raise RuntimeError(
                "aero_controller_msgs is required by hand_command_modes=script_service. "
                "Build and source seed_robot_ros2_pkg, or use joint_trajectory/disabled."
            ) from exc

    return _RosDeps(
        rclpy=rclpy,
        JointState=JointState,
        Image=Image,
        CompressedImage=CompressedImage,
        Twist=Twist,
        JointTrajectory=JointTrajectory,
        JointTrajectoryPoint=JointTrajectoryPoint,
        Duration=Duration,
        FollowJointTrajectory=FollowJointTrajectory,
        ActionClient=ActionClient,
        MultiThreadedExecutor=MultiThreadedExecutor,
        qos_profile_sensor_data=qos_profile_sensor_data,
        RunScript=run_script,
        ScriptReqJNoInterf=script_request,
    )


def _unique_ordered(values: list[str]) -> list[str]:
    return list(dict.fromkeys(values))



def _ros_image_to_rgb8(msg: Any) -> np.ndarray:
    """Convert sensor_msgs/Image into an HWC RGB uint8 NumPy array.
    Supported encodings:
      - rgb8
      - bgr8
      - rgba8
      - bgra8
      - mono8
    The ROS Image.step field is respected, including row padding.
    """
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    encoding = str(msg.encoding).lower()
    if height <= 0 or width <= 0:
        raise ValueError(
            f"Invalid ROS image dimensions: height={height}, width={width}"
        )
    source_channels = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "mono8": 1,
    }.get(encoding)
    if source_channels is None:
        raise ValueError(
            f"Unsupported ROS image encoding: {msg.encoding!r}. "
            "Supported encodings are rgb8, bgr8, rgba8, bgra8 and mono8."
        )
    required_row_bytes = width * source_channels
    if step < required_row_bytes:
        raise ValueError(
            f"ROS image step={step} is smaller than the required "
            f"{required_row_bytes} bytes for encoding={encoding!r}"
        )
    buffer = np.frombuffer(msg.data, dtype=np.uint8)
    required_size = height * step
    if buffer.size < required_size:
        raise ValueError(
            f"ROS image data is too short: received={buffer.size}, "
            f"required={required_size}"
        )
    # Respect row padding expressed by msg.step.
    rows = buffer[:required_size].reshape(height, step)
    pixels = rows[:, :required_row_bytes]
    image = pixels.reshape(height, width, source_channels)
    if encoding == "rgb8":
        rgb = image
    elif encoding == "bgr8":
        rgb = image[:, :, [2, 1, 0]]
    elif encoding == "rgba8":
        rgb = image[:, :, :3]
    elif encoding == "bgra8":
        rgb = image[:, :, [2, 1, 0]]
    elif encoding == "mono8":
        rgb = np.repeat(image, repeats=3, axis=2)
    else:
        raise AssertionError(f"Unhandled encoding: {encoding}")
    return np.ascontiguousarray(rgb, dtype=np.uint8)


def _ros_compressed_image_to_rgb8(msg: Any) -> np.ndarray:
    """Convert sensor_msgs/CompressedImage (JPEG/PNG) to HWC RGB uint8."""
    try:
        from PIL import Image as PILImage
    except ImportError as exc:
        raise RuntimeError(
            "Pillow is required for sensor_msgs/CompressedImage. "
            "Install it with: python -m pip install pillow"
        ) from exc

    try:
        with PILImage.open(BytesIO(bytes(msg.data))) as pil_image:
            image = np.asarray(pil_image.convert("RGB"), dtype=np.uint8)
    except Exception as exc:
        raise ValueError(
            f"Failed to decode compressed ROS image: format={getattr(msg, 'format', '')!r}"
        ) from exc

    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(
            f"Decoded compressed ROS image must have shape (H, W, 3), got {image.shape}"
        )

    return np.ascontiguousarray(image, dtype=np.uint8)


def _duration_msg(duration_cls: Any, seconds: float) -> Any:
    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1_000_000_000))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return duration_cls(sec=sec, nanosec=nanosec)


class _RosImageCameraProxy:
    """Minimal camera object so LeRobot counts ROS image streams as cameras.

    LeRobot's record pipeline uses ``len(robot.cameras)`` to size image writers.
    Exposing ROS image streams here makes direct ``lerobot-record`` compatible.
    """

    def __init__(self, robot: "SeedNoid", name: str, shape: tuple[int, int, int]):
        self._robot = robot
        self.name = name
        self.height, self.width, self.channels = shape
        self.is_connected = False

    def connect(self, warmup: bool = True) -> None:
        self.is_connected = True

    def disconnect(self) -> None:
        self.is_connected = False

    def async_read(self, timeout_ms: float | None = None) -> np.ndarray:
        if not self.is_connected:
            raise ConnectionError(f"ROS image camera {self.name!r} is not connected")
        return self._robot._get_ros_image_observation(
            self.name,
            (self.height, self.width, self.channels),
            time.monotonic(),
        )

    def read(self, color_mode: Any | None = None) -> np.ndarray:
        return self.async_read()


class SeedNoid(Robot):
    """LeRobot Robot implementation backed by ROS 2 and ros2_control.

    LeRobot owns the record/rollout/async/RTC loop. This class only implements
    hardware I/O, feature ordering, controller distribution, and safety limits.
    """

    config_class = SeedNoidConfig
    name = "seed_noid"

    def __init__(self, config: SeedNoidConfig):
        super().__init__(config)
        self.config = config

        self._ros: _RosDeps | None = None
        self._node: Any | None = None
        self._executor: Any | None = None
        self._spin_thread: threading.Thread | None = None
        self._connected = False
        self._owns_rclpy_context = False

        self._joint_state_lock = threading.Lock()
        self._joint_positions: dict[str, float] = {}
        self._joint_state_stamp: float | None = None

        self._image_lock = threading.Lock()
        self._ros_images: dict[str, np.ndarray] = {}
        self._ros_image_stamps: dict[str, float] = {}

        self._joint_publishers: dict[str, Any] = {}
        self._joint_action_clients: dict[str, Any] = {}
        self._base_pub: Any | None = None
        self._hand_client: Any | None = None
        self._last_hand_state: dict[str, int | None] = {"rhand": None, "lhand": None}

        native_cameras = make_cameras_from_configs(config.cameras)
        ros_cameras = {
            name: _RosImageCameraProxy(self, name, tuple(config.ros_image_shapes[name]))
            for name in config.ros_image_topics
        }
        # Public attribute required by LeRobot record tooling.
        self.cameras = {**native_cameras, **ros_cameras}
        self._native_camera_names = set(native_cameras)

    @property
    def _observation_joint_names(self) -> list[str]:
        return _unique_ordered(
            [joint for group in self.config.observation_groups for joint in self.config.joint_groups[group]]
        )

    @property
    def _action_joint_names_by_group(self) -> dict[str, list[str]]:
        return {group: list(self.config.joint_groups[group]) for group in self.config.action_groups}

    @property
    def _command_joint_names_by_group(self) -> dict[str, list[str]]:
        return {group: list(self.config.joint_groups[group]) for group in self.config.command_groups}

    @property
    def _trajectory_command_groups(self) -> list[str]:
        groups: list[str] = []
        for group in self.config.command_groups:
            if group in ("rhand", "lhand"):
                mode = self.config.hand_command_modes.get(group, "disabled")
                if mode != "joint_trajectory":
                    continue
            groups.append(group)
        return groups

    @property
    def _service_hand_groups(self) -> list[str]:
        return [
            hand
            for hand in ("rhand", "lhand")
            if hand in self.config.command_groups
            and self.config.hand_command_modes.get(hand, "disabled") == "script_service"
        ]

    @property
    def _effective_command_groups(self) -> set[str]:
        groups: set[str] = set()
        for group in self.config.command_groups:
            if group in ("rhand", "lhand") and self.config.hand_command_modes.get(group, "disabled") == "disabled":
                continue
            groups.add(group)
        return groups

    @property
    def _action_joint_names(self) -> list[str]:
        return _unique_ordered(
            [joint for group in self.config.action_groups for joint in self.config.joint_groups[group]]
        )

    @property
    def observation_features(self) -> dict:
        features: dict[str, Any] = {f"{joint}.pos": float for joint in self._observation_joint_names}
        for camera_name, camera in self.cameras.items():
            channels = int(getattr(camera, "channels", 3))
            features[camera_name] = (int(camera.height), int(camera.width), channels)
        return features

    @property
    def action_features(self) -> dict:
        features: dict[str, Any] = {f"{joint}.pos": float for joint in self._action_joint_names}
        if self.config.include_base_actions:
            features.update({"base.vx": float, "base.vy": float, "base.wz": float})
        return features

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        # ros2_control/controller configuration remains owned by the ROS launch stack.
        return None

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            return

        self._ros = _load_ros_deps(
            require_hand_service=bool(self._service_hand_groups),
        )
        rclpy = self._ros.rclpy
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy_context = True

        self._node = rclpy.create_node(self.config.ros_node_name)
        qos = self._ros.qos_profile_sensor_data
        self._node.create_subscription(
            self._ros.JointState,
            self.config.joint_state_topic,
            self._on_joint_state,
            qos,
        )

        if self.config.ros_image_topics:
            for name, topic in self.config.ros_image_topics.items():
                if topic.endswith("/compressed"):
                    self._node.create_subscription(
                        self._ros.CompressedImage,
                        topic,
                        lambda msg, camera_name=name: self._on_ros_compressed_image(
                            camera_name, msg
                        ),
                        qos,
                    )
                else:
                    self._node.create_subscription(
                        self._ros.Image,
                        topic,
                        lambda msg, camera_name=name: self._on_ros_image(
                            camera_name, msg
                        ),
                        qos,
                    )

        if self.config.joint_command_transport == "topic":
            for group, topic in self._joint_command_endpoints("topic").items():
                self._joint_publishers[group] = self._node.create_publisher(
                    self._ros.JointTrajectory, topic, 10
                )
        else:
            for group, action_name in self._joint_command_endpoints("action").items():
                client = self._ros.ActionClient(self._node, self._ros.FollowJointTrajectory, action_name)
                if self.config.wait_for_action_servers_s > 0 and not client.wait_for_server(
                    timeout_sec=self.config.wait_for_action_servers_s
                ):
                    raise TimeoutError(f"Action server not available: {action_name}")
                self._joint_action_clients[group] = client

        if self.config.include_base_actions:
            self._base_pub = self._node.create_publisher(self._ros.Twist, self.config.base_twist_topic, 10)

        if self._service_hand_groups:
            self._hand_client = self._node.create_client(self._ros.RunScript, self.config.hand_service_name)
            if self.config.hand_script_timeout_s > 0 and not self._hand_client.wait_for_service(
                timeout_sec=self.config.hand_script_timeout_s
            ):
                raise TimeoutError(f"Hand service not available: {self.config.hand_service_name}")

        self._executor = self._ros.MultiThreadedExecutor(num_threads=self.config.executor_threads)
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin,
            name="seed_noid_lerobot_ros_spin",
            daemon=True,
        )
        self._spin_thread.start()

        for camera in self.cameras.values():
            camera.connect()

        self._wait_for_initial_joint_state()
        self._wait_for_initial_ros_images()
        self._connected = True
        self._log_startup_summary()

    def disconnect(self) -> None:
        for camera in self.cameras.values():
            try:
                if getattr(camera, "is_connected", False):
                    camera.disconnect()
            except Exception:
                pass

        if self._executor is not None:
            try:
                self._executor.shutdown()
            except Exception:
                pass
        if self._spin_thread is not None and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
        if self._owns_rclpy_context and self._ros is not None:
            try:
                self._ros.rclpy.shutdown()
            except Exception:
                pass

        self._node = None
        self._executor = None
        self._spin_thread = None
        self._joint_publishers.clear()
        self._joint_action_clients.clear()
        self._base_pub = None
        self._hand_client = None
        self._connected = False
        self._owns_rclpy_context = False

    def get_observation(self) -> RobotObservation:
        self._assert_connected()
        now = time.monotonic()
        obs: dict[str, Any] = {}

        with self._joint_state_lock:
            positions = dict(self._joint_positions)
            stamp = self._joint_state_stamp
        if stamp is None or (now - stamp) > self.config.stale_joint_state_s:
            if self.config.missing_joint_policy == "raise":
                raise TimeoutError("No fresh joint_states message has been received.")

        for joint in self._observation_joint_names:
            key = f"{joint}.pos"
            if joint in positions:
                obs[key] = float(positions[joint])
            elif self.config.missing_joint_policy == "zero":
                obs[key] = 0.0
            elif self.config.missing_joint_policy == "nan":
                obs[key] = float("nan")
            else:
                raise KeyError(f"Joint {joint!r} is missing from {self.config.joint_state_topic}")

        for camera_name, camera in self.cameras.items():
            obs[camera_name] = camera.async_read()

        return obs

    def send_action(self, action: RobotAction) -> RobotAction:
        self._assert_connected()
        action_dict = {str(key): self._as_float(value) for key, value in dict(action).items()}
        self._validate_action(action_dict)

        with self._joint_state_lock:
            current_positions = dict(self._joint_positions)

        safe_joint_action: dict[str, float] = {}
        effective_groups = self._effective_command_groups
        for group, joints in self._action_joint_names_by_group.items():
            hand_mode = self.config.hand_command_modes.get(group, "joint_trajectory")
            for joint in joints:
                key = f"{joint}.pos"
                target = action_dict[key]
                current = current_positions.get(joint)
                if group not in effective_groups:
                    safe_joint_action[key] = float(current) if current is not None else target
                elif group in ("rhand", "lhand") and hand_mode == "script_service":
                    # Script thresholds apply to the policy's raw thumb command; a
                    # per-step joint delta clamp would prevent the threshold crossing.
                    safe_joint_action[key] = self._clip_joint_absolute(joint, target)
                else:
                    safe_joint_action[key] = self._clip_joint_target(joint, target, current)

        trajectory_action = {
            key: value
            for key, value in safe_joint_action.items()
            if self._group_for_joint(key.removesuffix(".pos")) in self._trajectory_command_groups
        }
        if trajectory_action:
            if self.config.joint_command_transport == "topic":
                self._publish_joint_trajectories(trajectory_action)
            else:
                self._send_joint_trajectory_goals(trajectory_action)

        for hand in self._service_hand_groups:
            joint = self.config.joint_groups[hand][0]
            self._send_hand_script_from_value(hand, safe_joint_action[f"{joint}.pos"])

        sent: dict[str, float] = dict(safe_joint_action)
        if self.config.include_base_actions:
            sent.update(self._send_base_action(action_dict))
        return sent

    def _on_joint_state(self, msg: Any) -> None:
        positions = {name: float(value) for name, value in zip(msg.name, msg.position, strict=False)}
        with self._joint_state_lock:
            self._joint_positions.update(positions)
            self._joint_state_stamp = time.monotonic()

    def _on_ros_image(self, name: str, msg: Any) -> None:
        try:
            image = _ros_image_to_rgb8(msg)
        except Exception as exc:
            if self._node is not None:
                self._node.get_logger().error(f"Failed to convert ROS image {name!r}: {exc}")
            return
        
        expected_shape = tuple(self.config.ros_image_shapes[name])

        if tuple(image.shape) != expected_shape:
            if self._node is not None:
                self._node.get_logger().error(f"ROS image {name!r} has shape {image.shape}, " f"expected {expected_shape}")
            return
            
        with self._image_lock:
            self._ros_images[name] = image
            self._ros_image_stamps[name] = time.monotonic()

    def _on_ros_compressed_image(self, name: str, msg: Any) -> None:
        try:
            image = _ros_compressed_image_to_rgb8(msg)
        except Exception as exc:
            if self._node is not None:
                self._node.get_logger().error(
                    f"Failed to convert compressed ROS image {name!r}: {exc}"
                )
            return

        expected_shape = tuple(self.config.ros_image_shapes[name])
        if tuple(image.shape) != expected_shape:
            if self._node is not None:
                self._node.get_logger().error(
                    f"Compressed ROS image {name!r} has shape {image.shape}, "
                    f"expected {expected_shape}"
                )
            return

        with self._image_lock:
            self._ros_images[name] = image
            self._ros_image_stamps[name] = time.monotonic()

    def _joint_command_endpoints(self, transport: str) -> dict[str, str]:
        endpoints: dict[str, str] = {}
        for group in self._trajectory_command_groups:
            if transport == "topic" and group in self.config.joint_trajectory_topics:
                endpoints[group] = self.config.joint_trajectory_topics[group]
                continue
            if transport == "action" and group in self.config.follow_joint_trajectory_actions:
                endpoints[group] = self.config.follow_joint_trajectory_actions[group]
                continue
            controller = self.config.controller_names.get(group)
            if controller is None:
                raise KeyError(f"No controller configured for command group {group!r}")
            suffix = "joint_trajectory" if transport == "topic" else "follow_joint_trajectory"
            endpoints[group] = f"/{controller}/{suffix}"
        return endpoints

    def _publish_joint_trajectories(self, joint_action: dict[str, float]) -> None:
        for group in self._trajectory_command_groups:
            names, positions = self._group_goal(group, joint_action)
            if not names:
                continue
            self._joint_publishers[group].publish(self._make_trajectory_msg(names, positions))

    def _send_joint_trajectory_goals(self, joint_action: dict[str, float]) -> None:
        for group in self._trajectory_command_groups:
            names, positions = self._group_goal(group, joint_action)
            if not names:
                continue
            goal = self._ros.FollowJointTrajectory.Goal()
            goal.trajectory = self._make_trajectory_msg(names, positions)
            self._joint_action_clients[group].send_goal_async(goal)

    def _group_goal(self, group: str, joint_action: dict[str, float]) -> tuple[list[str], list[float]]:
        names: list[str] = []
        positions: list[float] = []
        for joint in self.config.joint_groups[group]:
            key = f"{joint}.pos"
            if key in joint_action:
                names.append(joint)
                positions.append(float(joint_action[key]))
        return names, positions

    def _make_trajectory_msg(self, names: list[str], positions: list[float]) -> Any:
        msg = self._ros.JointTrajectory()
        msg.joint_names = names
        point = self._ros.JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = _duration_msg(self._ros.Duration, self.config.trajectory_duration_s)
        msg.points = [point]
        return msg

    def _clip_joint_target(self, joint: str, target: float, current: float | None) -> float:
        if current is not None and self.config.max_relative_target is not None:
            limit = self._relative_limit(joint)
            if limit is not None:
                target = min(max(target, current - limit), current + limit)
        return self._clip_joint_absolute(joint, target)

    def _clip_joint_absolute(self, joint: str, target: float) -> float:
        if joint in self.config.joint_position_limits:
            low, high = self.config.joint_position_limits[joint]
            target = min(max(target, float(low)), float(high))
        return float(target)

    def _relative_limit(self, joint: str) -> float | None:
        value = self.config.max_relative_target
        if value is None:
            return None
        if isinstance(value, dict):
            raw = value.get(joint, value.get(f"{joint}.pos"))
            return None if raw is None else float(raw)
        return float(value)

    def _send_hand_script_from_value(self, hand: str, value: float) -> None:
        desired: int | None
        if value >= self.config.hand_grasp_threshold:
            desired = 1
        elif value <= self.config.hand_release_threshold:
            desired = 0
        else:
            desired = None
        if desired is None or desired == self._last_hand_state.get(hand):
            return
        self._call_hand_script(hand, grasp=bool(desired))
        self._last_hand_state[hand] = desired

    def _call_hand_script(self, hand: str, *, grasp: bool) -> None:
        if self._hand_client is None:
            return
        cfg = self.config.hand_scripts[hand]
        script_no = cfg["grasp_script_no"] if grasp else cfg["release_script_no"]
        request = self._ros.RunScript.Request()
        request.jname_interf = []
        request.jno_interf = [
            self._ros.ScriptReqJNoInterf(
                msid=int(cfg["msid"]),
                send_no=int(cfg["send_no"]),
                script_no=int(script_no),
                arg=0,
                dio_run=0,
            )
        ]
        request.timeout_sec = float(self.config.hand_script_timeout_s)
        self._hand_client.call_async(request)

    def _send_base_action(self, action: dict[str, float]) -> dict[str, float]:
        vx = self._clip_abs(action["base.vx"], self.config.base_velocity_limits.get("vx", math.inf))
        vy = self._clip_abs(action["base.vy"], self.config.base_velocity_limits.get("vy", math.inf))
        wz = self._clip_abs(action["base.wz"], self.config.base_velocity_limits.get("wz", math.inf))
        msg = self._ros.Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self._base_pub.publish(msg)
        return {"base.vx": vx, "base.vy": vy, "base.wz": wz}

    def _validate_action(self, action: dict[str, float]) -> None:
        expected = set(self.action_features)
        missing = sorted(expected - set(action))
        if missing and self.config.strict_action_keys:
            raise KeyError(f"Action is missing expected features: {missing}")
        for key in expected:
            if key not in action:
                action[key] = 0.0
        invalid = {key: value for key, value in action.items() if not math.isfinite(value)}
        if invalid:
            raise ValueError(f"Action contains NaN/Inf: {invalid}")

    def _group_for_joint(self, joint: str) -> str:
        for group, joints in self._action_joint_names_by_group.items():
            if joint in joints:
                return group
        raise KeyError(joint)

    def _get_ros_image_observation(
        self, name: str, shape: tuple[int, int, int], now: float
    ) -> np.ndarray:
        with self._image_lock:
            image = self._ros_images.get(name)
            stamp = self._ros_image_stamps.get(name)
        if image is None or stamp is None or (now - stamp) > self.config.stale_image_s:
            if self.config.missing_image_policy == "raise":
                raise TimeoutError(f"No fresh ROS image received for {name!r}")
            return np.zeros(shape, dtype=np.uint8)
        if tuple(image.shape) != shape:
            raise ValueError(f"ROS image {name!r} has shape {image.shape}, expected {shape}")
        return image.astype(np.uint8, copy=False)

    def _wait_for_initial_joint_state(self) -> None:
        deadline = time.monotonic() + self.config.wait_for_joint_state_s
        while time.monotonic() < deadline:
            with self._joint_state_lock:
                if self._joint_state_stamp is not None:
                    return
            time.sleep(0.02)
        if self.config.wait_for_joint_state_s > 0 and self.config.missing_joint_policy == "raise":
            raise TimeoutError(f"No message received on {self.config.joint_state_topic}")

    def _wait_for_initial_ros_images(self) -> None:
        if not self.config.ros_image_topics or self.config.wait_for_images_s <= 0:
            return
        deadline = time.monotonic() + self.config.wait_for_images_s
        expected = set(self.config.ros_image_topics)
        while time.monotonic() < deadline:
            with self._image_lock:
                if expected.issubset(self._ros_images):
                    return
            time.sleep(0.02)
        if self.config.missing_image_policy == "raise":
            missing = sorted(expected - set(self._ros_images))
            raise TimeoutError(f"No image received for ROS cameras: {missing}")

    def _log_startup_summary(self) -> None:
        if self._node is None:
            return
        logger = self._node.get_logger()
        logger.info(
            f"LeRobot seed_noid connected: observation_dim={len(self._observation_joint_names)} "
            f"action_dim={len(self._action_joint_names)}, command_groups={self.config.command_groups}"
        )
        logger.info(f"Observation order: {self._observation_joint_names}")
        logger.info(f"Action order: {self._action_joint_names}")
        logger.info(
            f"Command transport={self.config.joint_command_transport}, "
            f"endpoints={self._joint_command_endpoints(self.config.joint_command_transport)}"
        )
        logger.info(f"Cameras: {list(self.cameras)}")

    def _assert_connected(self) -> None:
        if not self.is_connected:
            raise ConnectionError(f"{self} is not connected. Call connect() first.")

    @staticmethod
    def _as_float(value: Any) -> float:
        if isinstance(value, np.ndarray):
            if value.size != 1:
                raise ValueError(f"Expected scalar action, got ndarray shape={value.shape}")
            return float(value.item())
        try:
            # Handles torch scalar tensors without importing torch.
            if hasattr(value, "numel") and value.numel() == 1:
                return float(value.item())
        except Exception:
            pass
        return float(value)

    @staticmethod
    def _clip_abs(value: float, limit: float) -> float:
        if limit == math.inf:
            return float(value)
        return min(max(float(value), -abs(float(limit))), abs(float(limit)))


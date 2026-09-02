from __future__ import annotations

from dataclasses import dataclass, field

try:
    from lerobot.cameras import CameraConfig
except ImportError:  # pragma: no cover - compatibility with older LeRobot layouts
    from lerobot.cameras.configs import CameraConfig

try:
    from lerobot.robots.config import RobotConfig
except ImportError:  # pragma: no cover
    from lerobot.robots import RobotConfig


DEFAULT_JOINT_GROUPS: dict[str, list[str]] = {
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

# This order matches the existing 24-axis dataset/checkpoint contract.
DEFAULT_24AXIS_GROUP_ORDER: list[str] = [
    "rarm",
    "rhand",
    "larm",
    "lhand",
    "waist",
    "lifter",
    "head",
]

DEFAULT_CONTROLLER_NAMES: dict[str, str] = {
    "rarm": "rarm_controller",
    "rhand": "rhand_controller",
    "larm": "larm_controller",
    "lhand": "lhand_controller",
    "waist": "waist_controller",
    "lifter": "lifter_controller",
    "head": "head_controller",
}


def _copy_joint_groups() -> dict[str, list[str]]:
    return {name: list(joints) for name, joints in DEFAULT_JOINT_GROUPS.items()}


def _canonical_camera_name(name: str) -> str:
    prefix = "observation.images."
    return name[len(prefix) :] if name.startswith(prefix) else name


@RobotConfig.register_subclass("seed_noid")
@dataclass(kw_only=True)
class SeedNoidConfig(RobotConfig):
    """Configuration for the SEED/NOID LeRobot Robot plugin.

    ``action_groups`` defines the feature contract exposed to LeRobot. It defaults
    to the existing 24-axis order. ``command_groups`` is an independent safety
    gate that determines which controller groups are actually commanded.
    """

    # ROS 2 node and core topics.
    ros_node_name: str = "lerobot_seed_noid"
    joint_state_topic: str = "/joint_states"
    base_twist_topic: str = "/mechanum_controller/cmd_vel_teleop"
    hand_service_name: str = "/aero_controller/run_script"

    # Fixed state/action contract. List order is semantically significant.
    joint_groups: dict[str, list[str]] = field(default_factory=_copy_joint_groups)
    observation_groups: list[str] = field(default_factory=lambda: list(DEFAULT_24AXIS_GROUP_ORDER))
    action_groups: list[str] = field(default_factory=lambda: list(DEFAULT_24AXIS_GROUP_ORDER))

    # Safety gate: expose 24 action features, but only command these groups.
    command_groups: list[str] = field(default_factory=lambda: list(DEFAULT_24AXIS_GROUP_ORDER))

    # ROS 2 controller mapping.
    controller_names: dict[str, str] = field(default_factory=lambda: dict(DEFAULT_CONTROLLER_NAMES))
    joint_command_transport: str = "topic"
    joint_trajectory_topics: dict[str, str] = field(default_factory=dict)
    follow_joint_trajectory_actions: dict[str, str] = field(default_factory=dict)
    trajectory_duration_s: float = 0.60
    wait_for_action_servers_s: float = 5.0
    executor_threads: int = 2

    # Joint safety. All values are radians.
    joint_position_limits: dict[str, tuple[float, float]] = field(default_factory=dict)
    max_relative_target: float | dict[str, float] | None = 0.04
    strict_action_keys: bool = True

    # Observation freshness and startup waits.
    wait_for_joint_state_s: float = 10.0
    wait_for_images_s: float = 15.0
    stale_joint_state_s: float = 2.0
    missing_joint_policy: str = "raise"

    # Optional mobile-base action features.
    include_base_actions: bool = False
    base_velocity_limits: dict[str, float] = field(
        default_factory=lambda: {"vx": 0.20, "vy": 0.15, "wz": 0.40}
    )

    # Each hand can use its JointTrajectoryController, the existing RunScript
    # service, or be disabled while retaining the 24-axis feature contract.
    hand_command_modes: dict[str, str] = field(
        default_factory=lambda: {"rhand": "joint_trajectory", "lhand": "joint_trajectory"}
    )
    hand_grasp_threshold: float = 0.75
    hand_release_threshold: float = 0.25
    hand_script_timeout_s: float = 5.0
    hand_scripts: dict[str, dict[str, int]] = field(
        default_factory=lambda: {
            "rhand": {"msid": 1, "send_no": 11, "grasp_script_no": 2, "release_script_no": 3},
            "lhand": {"msid": 1, "send_no": 26, "grasp_script_no": 2, "release_script_no": 3},
        }
    )

    # ROS Image subscriptions. Keys are unprefixed LeRobot camera names, e.g.
    # ``camera1``. ``observation.images.`` is added by LeRobot's dataset tools.
    ros_image_topics: dict[str, str] = field(default_factory=lambda: {"head": "/camera1/image_raw/compressed", "right": "/camera2/image_raw/compressed", "left": "/camera3/image_raw/compressed",})
    ros_image_shapes: dict[str, tuple[int, int, int]] = field(default_factory=lambda: {"head": (480, 640, 3), "right": (480, 640, 3), "left": (480, 640, 3),})
    stale_image_s: float = 2.0
    missing_image_policy: str = "raise"

    # Native LeRobot cameras can be mixed with ROS image topics.
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    def __post_init__(self) -> None:
        super().__post_init__()

        self.ros_image_topics = {
            _canonical_camera_name(str(name)): str(topic) for name, topic in self.ros_image_topics.items()
        }
        self.ros_image_shapes = {
            _canonical_camera_name(str(name)): tuple(int(v) for v in shape)
            for name, shape in self.ros_image_shapes.items()
        }

        unknown_observation = [group for group in self.observation_groups if group not in self.joint_groups]
        unknown_action = [group for group in self.action_groups if group not in self.joint_groups]
        unknown_command = [group for group in self.command_groups if group not in self.action_groups]
        if unknown_observation:
            raise ValueError(f"Unknown observation_groups: {unknown_observation}")
        if unknown_action:
            raise ValueError(f"Unknown action_groups: {unknown_action}")
        if unknown_command:
            raise ValueError(f"command_groups must be a subset of action_groups: {unknown_command}")

        for name, groups in (
            ("observation_groups", self.observation_groups),
            ("action_groups", self.action_groups),
            ("command_groups", self.command_groups),
        ):
            if len(groups) != len(set(groups)):
                raise ValueError(f"{name} contains duplicates: {groups}")

        observation_joints = [joint for group in self.observation_groups for joint in self.joint_groups[group]]
        action_joints = [joint for group in self.action_groups for joint in self.joint_groups[group]]
        if len(observation_joints) != len(set(observation_joints)):
            raise ValueError("A joint appears more than once in the observation feature order.")
        if len(action_joints) != len(set(action_joints)):
            raise ValueError("A joint appears more than once in the action feature order.")

        if set(self.ros_image_topics) != set(self.ros_image_shapes):
            missing_shapes = sorted(set(self.ros_image_topics) - set(self.ros_image_shapes))
            missing_topics = sorted(set(self.ros_image_shapes) - set(self.ros_image_topics))
            raise ValueError(
                "ros_image_topics and ros_image_shapes must have identical keys. "
                f"missing_shapes={missing_shapes}, missing_topics={missing_topics}"
            )

        native_names = set(self.cameras)
        ros_names = set(self.ros_image_topics)
        overlap = sorted(native_names & ros_names)
        if overlap:
            raise ValueError(f"Camera names are duplicated between cameras and ros_image_topics: {overlap}")

        for name, shape in self.ros_image_shapes.items():
            if len(shape) != 3 or shape[2] not in (1, 3):
                raise ValueError(f"ROS image {name!r} must have shape (H, W, 1|3), got {shape}")

        allowed_transprts = {"topic", "action"}
        if self.joint_command_transport not in allowed_transprts:
            raise ValueError("joint_command_transport must be one of " f"{sorted(allowed_transprts)}," f"got{self.joint_command_transport!r}")
        
        allowed_missing_joint_policies = {"nan", "zero", "raise"}
        if self.missing_joint_policy not in allowed_missing_joint_policies:
            raise ValueError("missing_joint_policy must be one of " f"{sorted(allowed_missing_joint_policies)}," f"got {self.missing_joint_policy!r}")

        allowed_missing_image_policies = {"zero", "raise"}
        if self.missing_image_policy not in allowed_missing_image_policies:
            raise ValueError("missing_image_policy must be one of " f"{sorted(allowed_missing_image_policies)}, " f"got {self.missing_image_policy!r}")

        allowed_transports = {"topic", "action"}
        if self.joint_command_transport not in allowed_transports:
            raise ValueError(
                "joint_command_transport must be one of "
                f"{sorted(allowed_transports)}, "
                f"got {self.joint_command_transport!r}"
            )

        allowed_missing_joint_policies = {"nan", "zero", "raise"}
        if self.missing_joint_policy not in allowed_missing_joint_policies:
            raise ValueError(
                "missing_joint_policy must be one of "
                f"{sorted(allowed_missing_joint_policies)}, "
                f"got {self.missing_joint_policy!r}"
            )

        allowed_missing_image_policies = {"zero", "raise"}
        if self.missing_image_policy not in allowed_missing_image_policies:
            raise ValueError(
                "missing_image_policy must be one of "
                f"{sorted(allowed_missing_image_policies)}, "
                f"got {self.missing_image_policy!r}"
            )

        allowed_hand_modes = {"joint_trajectory", "script_service", "disabled"}
        for hand in ("rhand", "lhand"):
            mode = self.hand_command_modes.get(hand, "disabled")
            if mode not in allowed_hand_modes:
                raise ValueError(f"Invalid hand_command_modes[{hand!r}]={mode!r}")
            if mode == "script_service" and hand not in self.hand_scripts:
                raise ValueError(f"Missing hand_scripts configuration for {hand}")

        if self.trajectory_duration_s <= 0:
            raise ValueError("trajectory_duration_s must be positive")
        if self.executor_threads < 1:
            raise ValueError("executor_threads must be >= 1")
        if isinstance(self.max_relative_target, float) and self.max_relative_target <= 0:
            raise ValueError("max_relative_target must be positive or None")
        if isinstance(self.max_relative_target, dict):
            invalid = {key: value for key, value in self.max_relative_target.items() if float(value) <= 0}
            if invalid:
                raise ValueError(f"max_relative_target values must be positive: {invalid}")


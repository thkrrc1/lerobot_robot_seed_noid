"""Run with pytest in an environment containing LeRobot; no ROS connection required."""

from lerobot_robot_seed_noid import SeedNoid, SeedNoidConfig


def test_default_24_axis_contract():
    cfg = SeedNoidConfig(id="test")
    robot = SeedNoid(cfg)
    assert len(robot.observation_features) == 24
    assert len(robot.action_features) == 24
    assert list(robot.action_features)[:8] == [
        "r_shoulder_p_joint.pos",
        "r_shoulder_r_joint.pos",
        "r_shoulder_y_joint.pos",
        "r_elbow_joint.pos",
        "r_wrist_y_joint.pos",
        "r_wrist_p_joint.pos",
        "r_wrist_r_joint.pos",
        "r_thumb_joint.pos",
    ]
    assert cfg.command_groups == ["rarm"]


def test_ros_camera_is_exposed_to_lerobot_camera_count():
    cfg = SeedNoidConfig(
        id="test_camera",
        ros_image_topics={"camera1": "/camera/image_raw"},
        ros_image_shapes={"camera1": (720, 1280, 3)},
    )
    robot = SeedNoid(cfg)
    assert "camera1" in robot.cameras
    assert robot.observation_features["camera1"] == (720, 1280, 3)

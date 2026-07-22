"""Connect through ROS 2 and print one LeRobot-formatted observation."""

from pprint import pprint

from lerobot_robot_seed_noid import SeedNoid, SeedNoidConfig

config = SeedNoidConfig(
    id="seed_noid_observation_test",
    ros_image_topics={"camera1": "/camera/camera/color/image_raw"},
    ros_image_shapes={"camera1": (720, 1280, 3)},
    command_groups=[],  # no controller publishers/actions are created
)

with SeedNoid(config) as robot:
    observation = robot.get_observation()
    pprint({key: getattr(value, "shape", value) for key, value in observation.items()})

"""Offline discovery/configuration check. ROS 2 connection is not attempted."""

from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.robots.utils import make_robot_from_config

from lerobot_robot_seed_noid import SeedNoidConfig

register_third_party_plugins()

config = SeedNoidConfig(
    id="seed_noid_check",
    ros_image_topics={"camera1": "/camera/camera/color/image_raw"},
    ros_image_shapes={"camera1": (720, 1280, 3)},
)
robot = make_robot_from_config(config)

print("robot:", robot)
print("observation features:", robot.observation_features)
print("action features:", robot.action_features)
print("action dimension:", len(robot.action_features))
print("command groups:", config.command_groups)

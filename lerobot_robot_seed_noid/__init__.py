"""LeRobot external Robot plugin for the SEED/NOID ROS 2 robot stack.

Importing this package registers ``robot.type=seed_noid`` with LeRobot.
"""

from .config_seed_noid import SeedNoidConfig
from .seed_noid import SeedNoid

__all__ = ["SeedNoid", "SeedNoidConfig"]

__version__ = "6.0.12"

import logging

logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

from .interface import run_ros_bridge, run_ros_bridge_main

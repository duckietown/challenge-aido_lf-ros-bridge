__version__ = "6.0.28"

import logging

logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

logger.debug(f"duckietown_aido_ros_bridge version {__version__} path {__file__}")
from .interface import run_ros_bridge, run_ros_bridge_main

__version__ = "6.0.20"

import logging

logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

logger.info(f"version {__version__}")
from .interface import run_ros_bridge, run_ros_bridge_main

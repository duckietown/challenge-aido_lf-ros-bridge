__version__ = "6.0.87"

import logging

logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
import os

path = os.path.dirname(os.path.dirname(__file__))


logger.debug(f"duckietown_aido_ros_bridge version {__version__} path {path}")
from .interface import run_ros_bridge, run_ros_bridge_main

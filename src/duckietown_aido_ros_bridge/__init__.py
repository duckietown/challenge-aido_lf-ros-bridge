__version__ = "6.0.3"

import logging

logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

from .interface import run_ros_bridge

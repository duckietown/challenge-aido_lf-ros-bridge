import argparse
import sys
import time
import traceback
from multiprocessing import Queue
from multiprocessing.context import Process
from queue import Queue

from aido_schemas import logger
from .agent import run_agent

from .bridge import run_bridge
from .launch import run_roslaunch

logger.setLevel(logger.DEBUG)
__all__ = ["run_ros_bridge", "run_ros_bridge_main"]


def run_ros_bridge_main(args=None):
    parser = argparse.ArgumentParser(args)
    parser.add_argument("--launch", required=True)
    parsed = parser.parse_args(args)
    launch = parsed.launch
    try:
        run_ros_bridge(launch)
    except:
        logger.error(traceback.format_exc())
        sys.exit(2)


def run_ros_bridge(launch_file: str):
    logger.info(f"run_ros_bridge launch_file = {launch_file}")
    q_images = Queue()
    q_commands = Queue()
    q_control = Queue()

    logger.info(f"starting run_ros_launch")
    p_roslaunch = Process(
        target=run_roslaunch, args=(q_control, launch_file,), name="roslaunch"
    )
    p_roslaunch.start()

    logger.info("waiting 10 seconds")
    time.sleep(10)

    logger.info(f"starting run_bridge")
    p_rosnode = Process(
        target=run_bridge, args=(q_control, q_images, q_commands), name="rosnode"
    )
    p_rosnode.start()

    logger.info(f"starting run_agent")
    p_agent = Process(
        target=run_agent, args=(q_control, q_images, q_commands), name="aido_agent"
    )
    p_agent.start()

    while True:
        logger.info(f"looking for responses")
        d = q_control.get(block=True)
        logger.info(f"obtained {d}")

    # noinspection PyUnreachableCode
    logger.info(f"waiting for agent to finish")
    p_agent.join()

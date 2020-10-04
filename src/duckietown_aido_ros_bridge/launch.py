import logging
import os
import subprocess
import sys
import time
import traceback

from duckietown_aido_ros_bridge.commons import wrap_for_errors


@wrap_for_errors
def run_roslaunch(q_control, launch_file: str):
    logging.basicConfig()
    logger = logging.getLogger("run_roslaunch")
    logger.setLevel(logging.DEBUG)

    logger.info("run_roslaunch started")
    my_env = os.environ.copy()
    # command = f"roslaunch {launch_file}"
    command = ["roslaunch", launch_file]
    logger.info(f"running {command}")
    os.system(" ".join(command))
    try:
        subprocess.check_call(
            command,  # env=my_env
        )  # , stdout=sys.stdout, stderr=sys.stderr)
    except subprocess.CalledProcessError as e:
        logger.error(traceback.format_exc())
        raise
    # p = subprocess.Popen(
    #     command, shell=True, env=my_env, stdout=sys.stdout, stderr=sys.stderr
    # )
    # p.communicate()
    logger.info("run_roslaunch ended")
    time.sleep(1000)

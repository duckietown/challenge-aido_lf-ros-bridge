import argparse
import os
import subprocess
import sys
import time
import traceback
from multiprocessing import Queue
from multiprocessing.context import Process
from queue import Empty, Queue

import numpy as np
import rospy
from aido_schemas import (
    Context,
    Duckiebot1Commands,
    Duckiebot1Observations,
    EpisodeStart,
    GetCommands,
    LEDSCommands,
    logger,
    protocol_agent_duckiebot1,
    PWMCommands,
    RGB,
    wrap_direct,
)
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import CameraInfo, CompressedImage

from .commons import compressed_img_from_rgb, rgb_from_jpg

logger.setLevel(logger.DEBUG)
__all__ = ["run_ros_bridge", "run_ros_bridge_main"]


class ROSBridge:
    pub_image: rospy.Publisher
    pub_camera_info: rospy.Publisher
    updated: bool
    action: np.ndarray

    def __init__(self, qcommands: Queue, qimages: Queue):
        self.qcommands = qcommands
        self.qimages = qimages

    def go(self):
        self.action = np.array([0.0, 0.0])
        self.updated = True

        # Get the vehicle name, which comes in as HOSTNAME
        vehicle = os.getenv("HOSTNAME")
        logger.info(f"Using vehicle = {vehicle}")

        topic = f"/{vehicle}/wheels_driver_node/wheels_cmd"
        logger.info(f"Subscribing to {topic}")
        rospy.Subscriber(topic, WheelsCmdStamped, self.on_ros_received_wheels_cmd)

        # Publishes onto the corrected image topic
        # since image out of simulator is currently rectified
        topic = f"/{vehicle}/camera_node/image/compressed"
        queue_size = 10
        logger.info(f"Preparing publisher to {topic}; queue_size = {queue_size}")
        self.pub_image = rospy.Publisher(topic, CompressedImage, queue_size=queue_size)

        # Publisher for camera info - needed for the ground_projection
        topic = f"/{vehicle}/camera_node/camera_info"
        queue_size = 1
        logger.info(f"Preparing publisher to {topic}; queue_size = {queue_size}")
        self.pub_camera_info = rospy.Publisher(topic, CameraInfo, queue_size=queue_size)

        def read_data(event):
            data: Duckiebot1Observations
            try:
                data = self.qimages.get(block=False, timeout=0.0)
            except Empty:
                return
            logger.info("Received observations")
            jpg_data = data.camera.jpg_data
            obs = rgb_from_jpg(jpg_data)

            img_msg = compressed_img_from_rgb(obs)
            logger.info("Publishing image to ROS")

            self.pub_image.publish(img_msg)
            logger.info("Publishing CameraInfo")
            self.pub_camera_info.publish(CameraInfo())

        # Initializes the node
        logger.info("Calling rospy.init_node()")
        rospy.init_node("ROSTemplate")
        logger.info("Calling rospy.init_node() successful")

        rospy.Timer(rospy.Duration(0.01), read_data)

    def on_ros_received_wheels_cmd(self, msg):
        """
        Callback to listen to last outputted action from inverse_kinematics node
        Stores it and sustains same action until new message published on topic
        """
        logger.info(f"Received wheels_cmd")
        vl = msg.vel_left
        vr = msg.vel_right
        self.qcommands.put([vl, vr])


class AIDOAgent:
    def __init__(self, qcommands: Queue, qimages: Queue):
        self.qcommands = qcommands
        self.qimages = qimages

    def init(self, context: Context):
        context.info("init()")

    def on_received_seed(self, context: Context, data: int):
        np.random.seed(data)

    def on_received_episode_start(self, context: Context, data: EpisodeStart):
        context.info("Starting episode %s." % data)
        # TODO should we reset things?

    def on_received_observations(self, context: Context, data: Duckiebot1Observations):
        context.info("Received observations")
        self.qimages.put(data)

    def on_received_get_commands(self, context: Context, data: GetCommands):
        context.info("Received request for GetCommands")
        MAX_WAIT = 2
        try:
            action = self.qcommands.get(block=True, timeout=MAX_WAIT)
        except Empty:
            msg = "Received no commands for {MAX_WAIT}s. Bailing out, using previous commands. "
            context.error(msg)
            return

        pwm_left, pwm_right = action

        grey = RGB(0.5, 0.5, 0.5)
        led_commands = LEDSCommands(grey, grey, grey, grey, grey)
        pwm_commands = PWMCommands(motor_left=pwm_left, motor_right=pwm_right)
        commands = Duckiebot1Commands(pwm_commands, led_commands)
        context.write("commands", commands)

    def finish(self, context):
        context.info("finish()")


def wrap_for_errors(f):
    def f2(q_control, *args, **kwargs):
        logger.info(f"{f.__name__} started")
        try:
            f(q_control, *args, **kwargs)
        except:
            logger.error(f"{f.__name__} terminated", traceback=traceback.format_exc())
            q_control.put((f.__name__, "error"))
            sys.exit(3)
        else:
            q_control.put((f.__name__, "finished"))
            logger.info(f"{f.__name__} terminated gracefully")

    return f2


@wrap_for_errors
def run_bridge(q_control, q_images, q_commands):
    logger.info("run_bridge started")
    bridge = ROSBridge(q_images, q_commands)
    bridge.go()
    logger.info("run_bridge ended")
    time.sleep(1000)


@wrap_for_errors
def run_agent(q_control, q_images, q_commands, q_init):
    logger.info("run_agent started")
    agent = AIDOAgent(q_images, q_commands)
    wrap_direct(agent, protocol_agent_duckiebot1, args=[])
    logger.info("run_agent ended")
    time.sleep(1000)


@wrap_for_errors
def run_roslaunch(q_control, launch_file: str, q_init: Queue):
    logger.info("run_roslaunch started")
    my_env = os.environ.copy()
    command = f"roslaunch {launch_file}"
    logger.info("running", command=command)
    p = subprocess.Popen(
        command, shell=True, env=my_env, stdout=sys.stdout, stderr=sys.stderr
    )
    p.communicate()
    logger.info("run_roslaunch ended")
    time.sleep(1000)


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
    # q_init.get()
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
        d = q_control.get(block=True)
        logger.info(f"obtained {d}")

    logger.info(f"waiting for agent to finish")
    p_agent.join()

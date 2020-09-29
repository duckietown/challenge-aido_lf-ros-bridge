import os
import time

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

__all__ = ["run_ros_bridge"]


class ROSBridge:
    pub_image: rospy.Publisher
    pub_camera_info: rospy.Publisher
    updated: bool
    action: np.ndarray

    def __init__(self):
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

        # Initializes the node
        logger.info("Calling rospy.init_node()")
        rospy.init_node("ROSTemplate")
        logger.info("Calling rospy.init_node() successful")

    def init(self, context: Context):
        context.info("init()")

    def on_received_seed(self, context: Context, data: int):
        np.random.seed(data)

    def on_received_episode_start(self, context: Context, data: EpisodeStart):
        context.info("Starting episode %s." % data)
        # TODO should we reset things?

    def on_ros_received_wheels_cmd(self, msg):
        """
        Callback to listen to last outputted action from inverse_kinematics node
        Stores it and sustains same action until new message published on topic
        """
        logger.info(f"Received wheels_cmd")
        vl = msg.vel_left
        vr = msg.vel_right
        self.action = np.array([vl, vr])
        self.updated = True

    def on_received_observations(self, context: Context, data: Duckiebot1Observations):
        context.info("Received observations")
        jpg_data = data.camera.jpg_data
        obs = rgb_from_jpg(jpg_data)

        img_msg = compressed_img_from_rgb(obs)
        context.info("Publishing image to ROS")

        self.pub_image.publish(img_msg)

        if not self.updated:
            vl = vr = 0
            self.action = np.array([vl, vr])
            self.updated = True
        context.info("Publishing CameraInfo")
        self.pub_camera_info.publish(CameraInfo())

    def on_received_get_commands(self, context: Context, data: GetCommands):
        context.info("Received request for GetCommands")
        # TODO: let's use a queue here. Performance suffers otherwise.
        # What you should do is: *get the last command*, if available
        # otherwise, wait for one command.
        t0 = time.time()
        MAX_WAIT = 2
        while not self.updated:
            context.info("Not update: await")
            time.sleep(0.05)
            dt = time.time() - t0
            if dt > MAX_WAIT:
                msg = "Received no commands for {MAX_WAIT}s. Bailing out, using previous commands. "
                context.error(msg)
                break

        pwm_left, pwm_right = self.action
        self.updated = False

        grey = RGB(0.5, 0.5, 0.5)
        led_commands = LEDSCommands(grey, grey, grey, grey, grey)
        pwm_commands = PWMCommands(motor_left=pwm_left, motor_right=pwm_right)
        commands = Duckiebot1Commands(pwm_commands, led_commands)

        context.write("commands", commands)

    def finish(self, context):
        context.info("finish()")


def run_ros_bridge():
    agent = ROSBridge()
    wrap_direct(agent, protocol_agent_duckiebot1)

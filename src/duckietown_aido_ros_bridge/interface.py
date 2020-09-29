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

    def init(self, context: Context):
        context.info("init()")

        # Get the vehicle name, which comes in as HOSTNAME
        vehicle = os.getenv("HOSTNAME")
        topic = f"/{vehicle}/wheels_driver_node/wheels_cmd"
        rospy.Subscriber(topic, WheelsCmdStamped, self.on_ros_received_wheels_cmd)

        self.action = np.array([0.0, 0.0])
        self.updated = True

        # Publishes onto the corrected image topic
        # since image out of simulator is currently rectified
        topic = f"/{vehicle}/camera_node/image/compressed"
        self.pub_image: rospy.Publisher(topic, CompressedImage, queue_size=10)

        # Publisher for camera info - needed for the ground_projection
        topic = f"/{vehicle}/camera_node/camera_info"
        self.pub_camera_info = rospy.Publisher(topic, CameraInfo, queue_size=1)

        # Initializes the node
        rospy.init_node("ROSTemplate")

        # 15Hz ROS Cycle - TODO: What is this number?
        self.r = rospy.Rate(15)

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
        vl = msg.vel_left
        vr = msg.vel_right
        self.action = np.array([vl, vr])
        self.updated = True

    def on_received_observations(self, context: Context, data: Duckiebot1Observations):
        jpg_data = data.camera.jpg_data
        obs = rgb_from_jpg(jpg_data)

        img_msg = compressed_img_from_rgb(obs)
        self.pub_image.publish(img_msg)

        if not self.updated:
            vl = vr = 0
            self.action = np.array([vl, vr])
            self.updated = True

        self.pub_camera_info.publish(CameraInfo())

    def on_received_get_commands(self, context: Context, data: GetCommands):
        # TODO: let's use a queue here. Performance suffers otherwise.
        # What you should do is: *get the last command*, if available
        # otherwise, wait for one command.
        while not self.updated:
            time.sleep(0.01)

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

import logging
import os
import time
from queue import Empty, Queue

import numpy as np

import rospy
from aido_schemas import DB20Observations
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import CameraInfo, CompressedImage
from .commons import compressed_img_from_rgb, rgb_from_jpg, wrap_for_errors


class ROSBridge:
    pub_image: rospy.Publisher
    pub_camera_info: rospy.Publisher
    updated: bool
    action: np.ndarray

    def __init__(self, qcommands: Queue, qimages: Queue, logger):
        self.qcommands = qcommands
        self.qimages = qimages
        self.logger = logger

    def go(self):
        self.action = np.array([0.0, 0.0])
        self.updated = True

        # Initializes the node
        self.logger.info("Calling rospy.init_node()")
        rospy.init_node("ROSTemplate")
        rospy.loginfo("Calling rospy.init_node() successful")

        # Get the vehicle name, which comes in as HOSTNAME
        vehicle = os.getenv("HOSTNAME")
        self.logger.info(f"Using vehicle = {vehicle}")

        topic = f"/{vehicle}/wheels_driver_node/wheels_cmd"
        self.logger.info(f"Subscribing to {topic}")
        rospy.Subscriber(topic, WheelsCmdStamped, self.on_ros_received_wheels_cmd)

        # Publishes onto the corrected image topic
        # since image out of simulator is currently rectified
        topic = f"/{vehicle}/camera_node/image/compressed"
        queue_size = 10
        self.logger.info(f"Preparing publisher to {topic}; queue_size = {queue_size}")
        self.pub_image = rospy.Publisher(topic, CompressedImage, queue_size=queue_size)

        # Publisher for camera info - needed for the ground_projection
        topic = f"/{vehicle}/camera_node/camera_info"
        queue_size = 1
        self.logger.info(f"Preparing publisher to {topic}; queue_size = {queue_size}")
        self.pub_camera_info = rospy.Publisher(topic, CameraInfo, queue_size=queue_size)

        def read_data(event):
            self.logger.info("read_data called")
            data: DB20Observations
            try:
                data = self.qimages.get(block=False, timeout=0.0)
            except Empty:
                return
            self.logger.info("Received observations")
            jpg_data = data.camera.jpg_data
            obs = rgb_from_jpg(jpg_data)

            img_msg = compressed_img_from_rgb(obs)
            self.logger.info("Publishing image to ROS")

            self.pub_image.publish(img_msg)
            self.logger.info("Publishing CameraInfo")
            self.pub_camera_info.publish(CameraInfo())

        rospy.Timer(rospy.Duration(0.01), read_data)

        self.logger.info("spin()")
        rospy.spin()
        self.logger.info("spin() ended")

    def on_ros_received_wheels_cmd(self, msg):
        """
        Callback to listen to last outputted action from inverse_kinematics node
        Stores it and sustains same action until new message published on topic
        """
        self.logger.info(f"Received wheels_cmd")
        vl = msg.vel_left
        vr = msg.vel_right
        self.qcommands.put([vl, vr])


@wrap_for_errors
def run_bridge(q_control, q_images, q_commands):
    logging.basicConfig()
    logger = logging.getLogger("run_roslaunch")
    logger.setLevel(logging.DEBUG)

    logger.info("run_bridge started")
    bridge = ROSBridge(q_images, q_commands, logger)
    bridge.go()
    logger.info("run_bridge ended")
    time.sleep(1000)

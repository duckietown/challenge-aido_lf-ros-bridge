import io

import cv2
import numpy as np
import rospy
from PIL import Image
from sensor_msgs.msg import CompressedImage


def rgb_from_jpg(image_data: bytes) -> np.ndarray:
    """ Reads JPG bytes as RGB"""
    im = Image.open(io.BytesIO(image_data))
    im = im.convert("RGB")
    data = np.array(im)
    assert data.ndim == 3
    assert data.dtype == np.uint8
    return data


def compressed_img_from_rgb(rgb: np.ndarray) -> CompressedImage:
    img_msg = CompressedImage()

    time = rospy.get_rostime()
    img_msg.header.stamp.secs = time.secs
    img_msg.header.stamp.nsecs = time.nsecs

    img_msg.format = "jpeg"
    contig = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_BGR2RGB)
    img_msg.data = np.array(cv2.imencode(".jpg", contig)[1]).tostring()
    return img_msg

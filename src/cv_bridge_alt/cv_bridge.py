"""
    Provides conversions between OpenCV and ROS image formats in a hard-coded way.  
    CV_Bridge, the module usually responsible for doing this, is not compatible with Python 3,
     - the language this all is written in.  So we create this module, and all is... well, all is not well,
     - but all works.  :-/

     DEPRECATED FOR OUR PURPOSES
"""
import sys
import numpy as np
import cv2 as cv
import rospy
from sensor_msgs.msg import Image

def bgraimgmsg_to_cv2(img_msg):

    # TODO: How do we handle conversion of bgra8 format from camera to rgb for YOLOP
    if img_msg.encoding != "bgra8":
        rospy.logerr("This Coral detect node has been hardcoded to the 'bgra8' encoding.  Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv_bgra = np.ndarray(shape=(img_msg.height, img_msg.width, 4), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)

    # TODO: I don't think I need to do this?
    # This handles conversion from BGRA to RGB
    # image_opencv = image_opencv_bgra[...,:3][...,::-1]
    # image_opencv = cv.cvtColor(image_opencv_bgra, cv.COLOR_BGRA2RGB)

    # This handles conversion from BGRA to BGR
    image_opencv = image_opencv_bgra[...,:3]

    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()

    # Make a writeable copy
    copy_image = np.copy(image_opencv)
    return copy_image

def depthimgmsg_to_cv2(img_msg):

    # TODO: How do we handle conversion of bgra8 format from camera to rgb for YOLOP
    if img_msg.encoding != "32FC1":
        rospy.logerr("This Coral detect node has been hardcoded to the 'bgra8' encoding.  Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("float") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv_depth = np.ndarray(
        shape=(img_msg.height, img_msg.width), # 1 Channel
        dtype=dtype,
        buffer=img_msg.data
    )

    # TODO: I don't think I need to do this?
    # This handles conversion from BGRA to RGB
    # image_opencv = image_opencv_bgra[...,:3][...,::-1]
    # image_opencv = cv.cvtColor(image_opencv_bgra, cv.COLOR_BGRA2RGB)

    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()

    # Make a writeable copy
    copy_image = np.copy(image_opencv_depth)
    return copy_image

# This isn't checked since I don't plan on using it
def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg
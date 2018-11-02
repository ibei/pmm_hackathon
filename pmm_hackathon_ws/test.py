#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8MultiArray
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import cv2
from std_msgs.msg import UInt8
from cv_bridge import CvBridge, CvBridgeError

def callback(data):
    
    print("ass")
    # print(type(data))
    cv_image = CvBridge().imgmsg_to_cv2(data, desired_encoding="passthrough")

    tmp = np.unique(cv_image)
    cv_image1 = cv2.merge([cv_image,cv_image,cv_image]).copy()
    temp = cv_image.shape
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s%s%s', tmp,temp,"ass")
    cv2.imshow("1",cv_image1)
    cv2.waitKey(1)

def callback1(data):
    
    print("ass1")
    # print(type(data))
    cv_image = CvBridge().imgmsg_to_cv2(data, desired_encoding="passthrough")

    tmp = np.unique(cv_image)
    # cv_image = cv2.merge([cv_image,cv_image,cv_image])
    temp = cv_image.shape
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s%s%s', tmp,temp,"ass1")
    cv2.imshow("i",cv_image)
    cv2.waitKey(1)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=False)
    # rospy.Subscriber('/xtion/rgb/image_raw', Image, callback1)
    rospy.Subscriber('/xtion/depth_registered/image_raw', Image, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
def listener1():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener1', anonymous=False)

    rospy.Subscriber('/xtion/rgb/image_raw', Image, callback1)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
if __name__ == '__main__':
    listener()
    # listener1()

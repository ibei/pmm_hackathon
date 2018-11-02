#!/usr/bin/env python

# Copyright (c) 2016 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Author:
#   * Sam Pfeiffer
#   * Job van Dieten
#   * Jordi Pages

import rospy
import time
import cv2
import random
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class UseImage(object):
  def __init__(self):
    rospy.loginfo("Using Image...")

    self.bridge = CvBridge()
    self.image_pub = rospy.Publisher("image_topic_2", Image)
    rospy.Subscriber("xtion/rgb/image_raw", Image,
                     self.display_image)


  def display_image(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows, cols, channels) = cv_image.shape
    if cols > 60 and rows > 60:
      cv2.circle(cv_image, (50, 50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

    # image = data.data
    # im_h = data.height
    # im_w = data.width
    # # data_len = len(image)
    # # rospy.loginfo("image here: "+str(image[0]))
    # print("image heigh: {}, image width: {}".format(im_h, im_w))

if __name__ == '__main__':
  rospy.init_node('use_image', anonymous=True)
  
  use_im = UseImage()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

  rospy.spin()


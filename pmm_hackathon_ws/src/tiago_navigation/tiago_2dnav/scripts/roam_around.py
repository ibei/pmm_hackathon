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
import random
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal, MoveBaseActionFeedback
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# def stop_node():
#   print("Time elapsed, Stoping node...")

class TiagoRoam(object):
  def __init__(self):
    rospy.loginfo("Roaming mode enabled...")
    
    self.base_goal_cmd = rospy.Publisher(
        '/move_base_simple/goal', PoseStamped, queue_size=1)
    
    rospy.Subscriber("mobile_base_controller/odom", Odometry,
                     self.get_robot_state)
    rospy.wait_for_message("mobile_base_controller/odom", Odometry, 5)
    self.robot_init_pose = self.robot_pose
    
    rospy.sleep(3.0)

    self.go_to_shelf()
    
    rospy.sleep(15.0)

    self.go_home()

    # # rospy.Subscriber("move_base/feedback", MoveBaseActionFeedback,
    # #                  self.base_fb_callback)

    # # rospy.Subscriber("move_base/goal", MoveBaseGoal,
    # #                  self.base_goal_callback)

    

    # self.roam_duration = rospy.get_param('~roam_duration')
    # self.x_bound = [-2.5, 3]
    # self.y_bound = [-4.5, 5.5]
    # # nudge robot here so as to have the move_base/feedback published
    # rospy.sleep(3.0)
    # # self.nudge_robot()
    # self.time_elapsed = 0
    # self.time_now = time.time()
    # self.roam_robot(5*60)
  
  def base_goal_callback(self, data):
    rospy.loginfo("base goal callback: "+str(data))

  def roam_robot(self, duration):
    rospy.loginfo("Roaming...")

    while self.time_elapsed<duration:
      x_move = random.random()*5 - 2.75
      y_move = random.random()*10 - 5
      self.time_elapsed += time.time() - self.time_now
      self.time_now = time.time()
      base_goal = PoseStamped()
      base_goal.header.frame_id = "map"
      base_goal.pose.position.x = self.robot_pose.position.x + x_move
      base_goal.pose.position.y = self.robot_pose.position.y + y_move
      base_goal.pose.position.z = 0.0
      base_goal.pose.orientation = self.robot_pose.orientation

      if base_goal.pose.position.x > self.x_bound[1] or base_goal.pose.position.x < self.x_bound[0]:
        base_goal.pose.position.x = 0.0
      if base_goal.pose.position.y > self.y_bound[1] or base_goal.pose.position.y < self.y_bound[0]:
        base_goal.pose.position.y = 0.0

      self.base_goal_cmd.publish(base_goal)
      rospy.sleep(10.0)

      # self.base_fb_callback()

      # rospy.loginfo(self.time_elapsed)
  
    # include a publish that allows the robot to go back home here:
    self.go_home()
    rospy.signal_shutdown("Time elapsed, Stoping node...")


  def go_home(self):
    base_goal = PoseStamped()
    base_goal.header.frame_id = "map"
    base_goal.pose = self.robot_init_pose
    self.base_goal_cmd.publish(base_goal)
    rospy.loginfo("Going back home...")
    rospy.sleep(10.0)
    rospy.loginfo("REACHED HOME...")
  
  def go_to_shelf(self):
    rospy.loginfo("Going to shelf...")
    base_goal = PoseStamped()
    base_goal.header.frame_id = "map"
    base_goal.pose.position.x = -1
    base_goal.pose.position.y = 0.4
    base_goal.pose.position.z = 0
    base_goal.pose.orientation.x = 0
    base_goal.pose.orientation.y = 0
    base_goal.pose.orientation.z = -1
    base_goal.pose.orientation.w = 0
    self.base_goal_cmd.publish(base_goal)
    rospy.loginfo("Done.")


  def get_robot_state(self, data):
    self.odom_header = data.header
    self.robot_pose = data.pose.pose
  
  def base_fb_callback(self, data):
    print(data)


  def nudge_robot(self):
    rospy.loginfo("Starting to roam...")
    base_goal = PoseStamped()
    base_goal.header.frame_id = "map"
    base_goal.pose.position.x = self.robot_pose.position.x
    base_goal.pose.position.y = self.robot_pose.position.y + 0.5
    base_goal.pose.position.z = self.robot_pose.position.z
    base_goal.pose.orientation = self.robot_pose.orientation
    self.base_goal_cmd.publish(base_goal)
    rospy.loginfo("Done.")
  
  # def nudge_robot(self):
  #   rospy.loginfo("Starting to roam...")
  #   base_goal = MoveBaseActionGoal()
  #   base_goal.header.frame_id = "map"
  #   base_goal.goal_id.id = "nudge"
  #   base_goal.goal.target_pose.header.stamp = rospy.Time()
  #   base_goal.goal.target_pose.header.frame_id = "map"
  #   base_goal.goal.target_pose.pose.position.x = self.robot_pose.position.x
  #   base_goal.goal.target_pose.pose.position.y = self.robot_pose.position.y + 0.5
  #   base_goal.goal.target_pose.pose.position.z = self.robot_pose.position.z
  #   base_goal.goal.target_pose.pose.orientation = self.robot_pose.orientation
  #   # base_goal.goal.target_pose.pose.orientation.x = 0
  #   # base_goal.goal.target_pose.pose.orientation.y = 0
  #   # base_goal.goal.target_pose.pose.orientation.z = 0
  #   # base_goal.goal.target_pose.pose.orientation.w = 0
  #   self.base_goal_cmd.publish(base_goal)
  #   rospy.loginfo("Done.")


if __name__ == '__main__':
  rospy.init_node('roam_around')
  
  tiago_roam = TiagoRoam()

  rospy.spin()


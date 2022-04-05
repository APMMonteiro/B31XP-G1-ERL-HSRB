#! /usr/bin/env python

import roslib
import rospy
import actionlib

from control.msg import SemanticGoalAction, SemanticGoalGoal

class TestActionServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('test_action', SemanticGoalAction, self.execute)

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('test_action_server')
  server = TestActionServer()
  rospy.spin()

#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import sys

import rospy
# Brings in the SimpleActionClient
import actionlib

from actionlib_msgs.msg import GoalStatus, GoalID, GoalStatusArray
from std_stamped_msgs.msg import StringStamped, StringAction, StringFeedback, StringResult, StringGoal, EmptyStamped
from std_stamped_msgs.srv import StringService, StringServiceResponse


class GoToMarkerClient():
    def __init__(self):
        self._action_name = "go_to_marker_server"
        client = actionlib.SimpleActionClient(self._action_name, StringAction)
        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()
        # Creates a goal to send to the action server.
        goal = StringGoal()
        goal.data = 100

        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()
        # Prints out the result of executing the action
        result = client.get_result()  # A waiting_pickupResult
        rospy.loginfo("Action {} complete! Result: {}".format(self._action_name, result))

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('go_to_marker_client')
        client = GoToMarkerClient()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

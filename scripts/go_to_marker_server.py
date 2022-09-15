#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# General library
import os
import sys
import rospy
import rospkg
import actionlib
import tf
import json
import yaml
import dynamic_reconfigure.client
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int16, Header
from math import pi, sqrt
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from actionlib_msgs.msg import GoalStatus, GoalID, GoalStatusArray
from std_stamped_msgs.msg import StringStamped, StringAction, StringFeedback, StringResult, StringGoal, EmptyStamped
from std_stamped_msgs.srv import StringService, StringServiceResponse
from std_srvs.srv import SetBool
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
common_func_dir = os.path.join(rospkg.RosPack().get_path('agv_common_library'), 'scripts')
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(rospkg.RosPack().get_path('agv_common_library'), 'release')
sys.path.insert(0, common_func_dir)

from module_manager import ModuleServer, ModuleClient, ModuleStatus
from common_function import (
    MIN_FLOAT,
    EnumString,
    lockup_pose,
    offset_pose_x,
    offset_pose_yaw,
    distance_two_pose,
    delta_angle,
    offset_pose_xy_theta,
    angle_two_pose
)
# Local library
import tf2_ros
from move_to_point.msg import MoveToPointAction, MoveToPointResult, MoveToPointGoal

class GoToMarker(object):
    CODE_ONLY = False
    # create messages that are used to publish feedback/result
    _feedback = StringFeedback()
    _result = StringResult()
    def __init__(self, *args, **kwargs):
        # Action led control
        self._action_name = "go_to_marker_server"
        self._as = actionlib.SimpleActionServer(self._action_name, StringAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo("Start action go to marker")

        self.go_to_point_client = actionlib.SimpleActionClient('move_to_point', MoveToPointAction)
        # Waits until the action server has started up and started
        # listening for goals.
        self.go_to_point_client.wait_for_server()
        rospy.loginfo("Wait move to point server done")


        # Publisher
        self.marker_pose_pub = rospy.Publisher("/marker_pose", PoseStamped, queue_size=5)
        self.marker_goal_pose_pub = rospy.Publisher("/marker_goal_pose", PoseStamped, queue_size=5)
        # Subscriber
        # rospy.Subscriber("/led_status", StringStamped, self.led_status_cb)

        # self.load_config(kwargs["config_file"])
        rospy.on_shutdown(self.shutdown)

        # Initial
        self.init_variable(*args, **kwargs)


    """
    #### ##    ## #### ######## ####    ###    ##
     ##  ###   ##  ##     ##     ##    ## ##   ##
     ##  ####  ##  ##     ##     ##   ##   ##  ##
     ##  ## ## ##  ##     ##     ##  ##     ## ##
     ##  ##  ####  ##     ##     ##  ######### ##
     ##  ##   ###  ##     ##     ##  ##     ## ##
    #### ##    ## ####    ##    #### ##     ## ########
    """

    def init_variable(self, *args, **kwargs):
        self.simulation = kwargs["simulation"]
        rospy.logdebug("simulation: {}".format(self.simulation))
        self.simulation_str = "true" if self.simulation else "false"
        # TF
        self.led_effect_file = kwargs["config_file"]

        # General variable
        self.marker_id = 100
        self.x_offset = 1
        self.y_offset = 0.5
        self.yaw_offset = 0.785

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()


    def shutdown(self):
        # Kill all node
        print("Shutdown!")

    """
    ######## ##     ## ##    ##  ######  ######## ####  #######  ##    ##
    ##       ##     ## ###   ## ##    ##    ##     ##  ##     ## ###   ##
    ##       ##     ## ####  ## ##          ##     ##  ##     ## ####  ##
    ######   ##     ## ## ## ## ##          ##     ##  ##     ## ## ## ##
    ##       ##     ## ##  #### ##          ##     ##  ##     ## ##  ####
    ##       ##     ## ##   ### ##    ##    ##     ##  ##     ## ##   ###
    ##        #######  ##    ##  ######     ##    ####  #######  ##    ##
    """
    def pub_tf_goal(self, base_frame):
        trans_msg =  TransformStamped()
        trans_msg.header.stamp = rospy.Time.now()
        trans_msg.header.frame_id = base_frame
        trans_msg.child_frame_id = "goal_marker_" + str(self.marker_id)

        trans_msg.transform.translation.x = self.y_offset
        trans_msg.transform.translation.y = 0
        trans_msg.transform.translation.z = self.x_offset

        q = quaternion_from_euler(-1.57, self.yaw_offset-1.57, 0)
        trans_msg.transform.rotation.x = q[0]
        trans_msg.transform.rotation.y = q[1]
        trans_msg.transform.rotation.z = q[2]
        trans_msg.transform.rotation.w = q[3]
        self.broadcaster.sendTransform(trans_msg)
        rospy.loginfo("broadcaster goal pose done")

    def pose_filter(self, pose_fiterred, new_pose, cnt, angle_robot_vs_detected, detected_angle_allow, detected_distance_allow, dis_threshold=0.1, ang_threshold=0.1):
        """[summary]

        Args:
            pose_fiterred (PoseStamped): pose fiterred
            new_pose (PoseStamped): new pose
            cnt (int): number of pose fiterred

        Returns:
            (pose_filtered, cnt)
        """
        # Type of variable
        if self.CODE_ONLY:
            pose_fiterred = PoseStamped()
            new_pose = PoseStamped()

        # Check threshold:
        dx = abs(pose_fiterred.pose.position.x - new_pose.pose.position.x)
        dy = abs(pose_fiterred.pose.position.y - new_pose.pose.position.y)

        orientation = pose_fiterred.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        new_orientation = new_pose.pose.orientation
        (new_roll, new_pitch, new_yaw) = euler_from_quaternion([new_orientation.x, new_orientation.y, new_orientation.z, new_orientation.w])

        dyaw = abs(new_yaw - yaw)

        angle_allow_check = abs(delta_angle(new_yaw, angle_robot_vs_detected))
        distance_allow_check = sqrt(new_pose.pose.position.x**2 + new_pose.pose.position.y**2)

        rospy.loginfo("Current angle robot vs detected pose: {}, angle_allow_check: {}, distance_allow_check: {}"
                        .format(round(new_yaw, 2),round(angle_allow_check, 2), round(distance_allow_check, 2)))

        if (dx < dis_threshold and dy < dis_threshold and dyaw < ang_threshold and angle_allow_check < detected_angle_allow or cnt == 0)\
                    and distance_allow_check < detected_distance_allow:
            pose_fiterred.header = new_pose.header
            pose_fiterred.pose.position.x = (pose_fiterred.pose.position.x * cnt + new_pose.pose.position.x)/(cnt + 1)
            pose_fiterred.pose.position.y = (pose_fiterred.pose.position.y * cnt + new_pose.pose.position.y)/(cnt + 1)
            pose_fiterred.pose.position.z = (pose_fiterred.pose.position.z * cnt + new_pose.pose.position.z)/(cnt + 1)
            pose_fiterred.pose.orientation.x = (pose_fiterred.pose.orientation.x * cnt + new_pose.pose.orientation.x)/(cnt + 1)
            pose_fiterred.pose.orientation.y = (pose_fiterred.pose.orientation.y * cnt + new_pose.pose.orientation.y)/(cnt + 1)
            pose_fiterred.pose.orientation.z = (pose_fiterred.pose.orientation.z * cnt + new_pose.pose.orientation.z)/(cnt + 1)
            pose_fiterred.pose.orientation.w = (pose_fiterred.pose.orientation.w * cnt + new_pose.pose.orientation.w)/(cnt + 1)
            pose_fiterred.header.stamp = rospy.Time.now() # TOCHECK: set stamp to fix transformPose error
            return pose_fiterred, cnt+1
        else:
            rospy.logwarn("dock_pose detected exceed error threshold (dx: %0.3f, dy: %0.3f, dyaw: %0.3f, cnt: %i)"%(dx, dy, dyaw, cnt))
            pose_fiterred.header.stamp = rospy.Time.now()
            return pose_fiterred, cnt

    """
     ######     ###    ##       ##       ########     ###     ######  ##    ##
    ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
    ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
    ##       ##     ## ##       ##       ########  ##     ## ##       #####
    ##       ######### ##       ##       ##     ## ######### ##       ##  ##
    ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
     ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
    """

    def execute_cb(self, goal):
        rospy.logwarn("Start go to marker action!")
        self.action_state = "INIT"
        # helper variables
        success = True
        if self._as.is_preempt_requested():
            rospy.loginfo("%s: Preempted" % self._action_name)
            self._as.set_preempted()
            success = False
            # Action Preempted
            self.action_state = "ACTION_PREEMPTED"
        # Read goal data
        # data = json.loads(goal.data)
        # self.marker_id = int(data["marker_id"])
        # self.x_offset = int(data["x_offset"])
        # self.y_offset = int(data["y_offset"])
        # self.yaw_offset = int(data["yaw_offset"])
        self.marker_id = int(goal.data)

        rospy.loginfo("Goal marker ID: {}".format(self.marker_id))
        try:
            # Broadcaster goal pose
            marker_trans = TransformStamped()
            if not self.tfBuffer.can_transform('odom', 'fiducial_'+ str(self.marker_id), rospy.Time(), rospy.Duration(5.0)):
                rospy.logerr("lookup tranform goal marker {} pose error".format(str(self.marker_id)))
                rospy.loginfo("%s: Preempted" % self._action_name)
                self._as.set_preempted()
                success = False
                return
            else:
                marker_trans = self.tfBuffer.lookup_transform('odom', 'fiducial_'+ str(self.marker_id), rospy.Time())
                marker_pose = PoseStamped()
                marker_pose.header.stamp = rospy.Time.now()
                marker_pose.pose.position = marker_trans.transform.translation
                marker_pose.pose.orientation = marker_trans.transform.rotation
                self.marker_pose_pub.publish(marker_pose)
                self.pub_tf_goal('fiducial_'+ str(self.marker_id))

            # Get goal pose with odom frame
            goal_trans = TransformStamped()
            if self.tfBuffer.can_transform('odom', 'goal_marker_'+ str(self.marker_id), rospy.Time(), rospy.Duration(5.0)):
                goal_trans = self.tfBuffer.lookup_transform('odom', 'goal_marker_'+ str(self.marker_id), rospy.Time())
            # rospy.loginfo(type(trans))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("lookup tranform goal marker {} pose error".format(str(self.marker_id)))
            rospy.loginfo("%s: Preempted" % self._action_name)
            self._as.set_preempted()
            success = False
            return

        rospy.loginfo("Get goal with odom done")

        # send goal
        goal = MoveToPointGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.pose.position = goal_trans.transform.translation
        goal.target_pose.pose.orientation = goal_trans.transform.rotation

        # publish marker goal pose
        marker_goal_pose = PoseStamped()
        marker_goal_pose.header.stamp = rospy.Time.now()
        marker_goal_pose.pose.position = goal.target_pose.pose
        self.marker_pose_pub.publish(marker_goal_pose)

        print(goal)
        self.go_to_point_client.send_goal(goal)

        # Waits for the server to finish performing the action.
        self.go_to_point_client.wait_for_result()
        # Prints out the result of executing the action
        result = self.go_to_point_client.get_result()
        print("Action move complete! {}".format(result))

        #   check that preempt has not been requested by the client
        if success:
            self._result.status = 3
            rospy.loginfo("%s: Succeeded" % self._action_name)
            self._as.set_succeeded(self._result)


    """
    ##        #######   #######  ########
    ##       ##     ## ##     ## ##     ##
    ##       ##     ## ##     ## ##     ##
    ##       ##     ## ##     ## ########
    ##       ##     ## ##     ## ##
    ##       ##     ## ##     ## ##
    ########  #######   #######  ##
    """

    def loop(self):
        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            # Check pickup food led publish status
            try:
                if self.tfBuffer.can_transform('map', 'aruco_test_1', rospy.Time(), rospy.Duration(3.0)):
                    trans = self.tfBuffer.lookup_transform('map', 'aruco_test_1', rospy.Time())
                # rospy.loginfo(type(trans))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("lookup tranform error")

            r.sleep()

def parse_opts():
    from optparse import OptionParser
    parser = OptionParser()
    parser.add_option("-s", "--simulation",
                    action="store_true", dest="simulation", default=False, help="type \"-s\" if simulation")
    parser.add_option("-d", "--ros_debug",
                    action="store_true", dest="log_debug", default=False, help="log_level=rospy.DEBUG")
    parser.add_option("-c", "--config_file", dest="config_file",
                    default=os.path.join(rospkg.RosPack().get_path('go_to_marker'), 'cfg', 'moving_param.yaml'))

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)


if __name__ == '__main__':
    (options, args) = parse_opts()
    log_level = None
    if options.log_debug:
        log_level = rospy.DEBUG
    rospy.init_node('go_to_marker_server', log_level=log_level)
    rospy.loginfo('Init node ' + rospy.get_name())
    GoToMarker(**vars(options))

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
import numpy as np
from tf.transformations import *
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
        self.node_init_done = False
        self.config_path = kwargs["config_path"]
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

        # Service
        self.aruco_detect_en = rospy.ServiceProxy('/enable_detections', SetBool)

        rospy.on_shutdown(self.shutdown)

        # Initial
        self.init_variable(*args, **kwargs)
        self.loop()


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
        """
        Init variable
        """
        self.simulation = kwargs["simulation"]
        rospy.logdebug("simulation: {}".format(self.simulation))
        self.simulation_str = "true" if self.simulation else "false"

        # General variable
        self.marker_id = 101
        self.x_offset = 0.8
        self.y_offset = 0.0
        self.yaw_offset = 3.14
        self.base_frame = "odom"

        self.marker_cnt_target = 10
        self.marker_success_target = 5
        self.max_marker_check = 30

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.get_marker_pose_result = False
        self.marker_pose        = PoseStamped()
        self.marker_goal_pose   = PoseStamped()
        self.marker_pose.header.frame_id        = self.base_frame
        self.marker_goal_pose.header.frame_id   = self.base_frame


        # Disable Aruco detect
        set_result = self.aruco_detect_en(False)
        rospy.loginfo("Aruco detect disable result: {}".format(str(set_result.success)))
        self.aruco_enable = False
        self.node_init_done = True
        rospy.loginfo("Init variable done")


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
    def get_goal_from_marker(self, marker_pose, x_offset, y_offset, yaw_offset):
        """"Creat goal pose from marker pose

        Args:
            marker_pose (pose)  : Pose of marker
            x_offset (int )    : Offset with X axis of goal with marker by met
            y_offset (int)   : Offset with Y axis of goal with marker by met
            yaw_offset (int) : Offset with Yaw angle of goal with marker by radian

        Returns:
            pose: Pose of goal
        """
        if self.CODE_ONLY:
            marker_pose = Pose()
        goal_pose = Pose()

        # Get matrix of marker to odom
        _transMarker2Odom = np.array(
            [marker_pose.position.x, marker_pose.position.y, 0])
        _rotMarker2Odom = np.array([marker_pose.orientation.x, marker_pose.orientation.y,
                                    marker_pose.orientation.z, marker_pose.orientation.w])
        (roll, pitch, yaw) = euler_from_quaternion(_rotMarker2Odom)
        quat_2d = quaternion_from_euler(1.57, 0, yaw)
        print("Quat of marker: {}".format(quat_2d))
        _matrixMarker2Odom = concatenate_matrices(translation_matrix(
            _transMarker2Odom), quaternion_matrix(quat_2d))
        print( "Matrix Marker to Odom:")
        print(_matrixMarker2Odom)

        # Get matrix of goal to marker
        _transGoal2Marker = np.array([y_offset, 0, x_offset])
        # _transGoal2Marker = np.array([0, 0, 0])
        q_transform = quaternion_from_euler(-1.57, yaw_offset-1.57, 0)
        # print("q_transform")
        # print(q_transform)
        _rotGoal2Marker = np.array([q_transform[0], q_transform[1],
                                    q_transform[2], q_transform[3]])
        _matrixGoal2Marker = concatenate_matrices(translation_matrix(
            _transGoal2Marker), quaternion_matrix(_rotGoal2Marker))
        # print( "translation_matrix:")
        # print(translation_matrix(_transGoal2Marker))
        # print( "quaternion_matrix")
        # print(quaternion_matrix(_rotGoal2Marker))
        # print( "Matrix Goal to Marker:")
        # print(_matrixGoal2Marker)

        # Get matrix of goal to odom
        _matrixGoal2Odom = np.matmul(_matrixMarker2Odom, _matrixGoal2Marker)
        print("matrix goal to odom \n",_matrixGoal2Odom)

        # Get pose from matrix
        goal_pose.position.x   = translation_from_matrix(_matrixGoal2Odom)[0]
        goal_pose.position.y   = translation_from_matrix(_matrixGoal2Odom)[1]
        goal_pose.position.z   = translation_from_matrix(_matrixGoal2Odom)[2]
        goal_pose.orientation.x = quaternion_from_matrix(_matrixGoal2Odom)[0]
        goal_pose.orientation.y = quaternion_from_matrix(_matrixGoal2Odom)[1]
        goal_pose.orientation.z = quaternion_from_matrix(_matrixGoal2Odom)[2]
        goal_pose.orientation.w = quaternion_from_matrix(_matrixGoal2Odom)[3]
        print("goal pose \n",goal_pose)

        return goal_pose

    def pose_filter(self, pose_fiterred, new_pose, cnt, detected_distance_allow, dis_threshold=0.1, ang_threshold=0.1):
        """
        Filter pose to avoid noise and bias

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

        # angle_allow_check = abs(delta_angle(new_yaw, angle_robot_vs_detected))
        distance_allow_check = sqrt(new_pose.pose.position.x**2 + new_pose.pose.position.y**2)

        # rospy.loginfo("Current angle robot vs detected pose: {}, angle_allow_check: {}, distance_allow_check: {}"
        #                 .format(round(new_yaw, 2),round(angle_allow_check, 2), round(distance_allow_check, 2)))

        if (dx < dis_threshold and dy < dis_threshold and dyaw < ang_threshold or cnt == 0)\
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
            print("pose filter {} : {}".format(cnt, pose_fiterred.pose.position.x))
            return pose_fiterred, cnt+1
        else:
            rospy.logwarn("dock_pose detected exceed error threshold (dx: %0.3f, dy: %0.3f, dyaw: %0.3f, cnt: %i)"%(dx, dy, dyaw, cnt))
            pose_fiterred.header.stamp = rospy.Time.now()
            return pose_fiterred, cnt

    def get_marker_pose(self, marker_id):
        """
        Get pose of marker
        Args:
            marker_id : (int) ID of aruco marker

        Returns:
            marker_pose: pose
            result : (Bool) True if get pose
        """
        result = False
        marker_pose = PoseStamped()
        marker_trans = TransformStamped()

        try:
            # Broadcaster goal pose

            if not self.tfBuffer.can_transform('odom', 'fiducial_'+ str(self.marker_id), rospy.Time(), rospy.Duration(4.0)):
                rospy.logerr("can't lookup tranform marker {} pose".format(str(self.marker_id)))
                result = False
            else:
                marker_trans = self.tfBuffer.lookup_transform('odom', 'fiducial_'+ str(self.marker_id), rospy.Time())
                marker_pose.header.stamp = rospy.Time.now()
                marker_pose.pose.position = marker_trans.transform.translation
                marker_pose.pose.orientation= marker_trans.transform.rotation
                self.marker_pose_pub.publish(marker_pose)
                result = True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("lookup tranform marker {} pose error".format(str(self.marker_id)))
            result = False
        return marker_pose, result




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
        if not self.node_init_done:
            rospy.logwarn("Node don't init finish!")
            return
        rospy.loginfo("Start go to marker action!")
        self.action_state = "INIT"
        # helper variables
        success = True
        if self._as.is_preempt_requested():
            rospy.loginfo("%s: Preempted" % self._action_name)
            self._as.set_preempted()
            success = False
            # Action Preempted
            self.action_state = "ACTION_PREEMPTED"
        # Read parameter
        # data = json.loads(goal.data)
        # self.marker_id = int(data["marker_id"])
        # self.x_offset = int(data["x_offset"])
        # self.y_offset = int(data["y_offset"])
        # self.yaw_offset = int(data["yaw_offset"])
        self.marker_id = int(goal.data)

        # Load file config
        goto_marker_cfg_file = os.path.join(self.config_path, "marker_config.json")
        with open(goto_marker_cfg_file) as j:
                path_dict = json.load(j)
                # print_debug("Go to marker path:\n{}".format(json.dumps(path_dict, indent=2)))
                # offset_x_from_detected = path_dict["marker_params"]["offset_x_from_detected"]
                # offset_y_from_detected = path_dict["marker_params"]["offset_y_from_detected"]
                # offset_yaw_from_detected = path_dict["marker_params"]["offset_yaw_from_detected"]
                angle_robot_vs_detected = path_dict["marker_params"]["angle_robot_vs_detected"]
                detected_distance_allow = path_dict["marker_params"]["detected_distance_allow"]
                detected_angle_allow = path_dict["marker_params"]["detected_angle_allow"]

        rospy.loginfo("Goal marker ID: {}".format(self.marker_id))
        # Enable aruco detect
        if not self.aruco_enable:
            set_result = self.aruco_detect_en(True)
            rospy.loginfo("Aruco detect enable result: {}".format(str(set_result.success)))
            self.aruco_enable = True
        # Get marker pose:

        marker_cnt = 0
        get_marker_cnt_total = 0
        filtered_cnt = 0
        dock_pose_filtered = PoseStamped()
        self.last_marker_receive = rospy.get_time()
        r = rospy.Rate(15)
        while not rospy.is_shutdown():
            # print("Keyboard", KeyboardInterrupt)
            # Get marker use tf listen
            marker_pose, get_marker_result = self.get_marker_pose(self.marker_id)
            # If get marker success
            if get_marker_result:
                (dock_pose_filtered, filtered_cnt) = self.pose_filter(dock_pose_filtered, marker_pose,
                                            filtered_cnt, detected_distance_allow)
            # Increase marker counter
            marker_cnt += 1
            get_marker_cnt_total += 1

            # Nếu check liên tiếp vài lần mà không được lần nào
            if marker_cnt > self.marker_cnt_target:
                rospy.logerr('Not detected Marker')
                # Đặt lại tham số
                marker_cnt = 0
                dock_pose_filtered = PoseStamped()
                filtered_cnt = 0
                self.last_marker_receive = rospy.get_time()
                continue
            # Nếu số lần check marker lặp lại nhiều hơn số lần tối đa cho phép
            if get_marker_cnt_total >= self.max_marker_check:
                rospy.logerr('Marker error too much')
                break
            rospy.loginfo("Check Marker: %i/%i"%(filtered_cnt, marker_cnt))

            # Check đủ số lần nhưng không vượt quá số lần tối đa
            if (marker_cnt >= self.marker_cnt_target and filtered_cnt >= self.marker_success_target):
                rospy.loginfo('Detected Marker')
                # Disable aruco detect service
                set_result = self.aruco_detect_en(False)
                rospy.loginfo("Aruco detect disable result: {}".format(str(set_result.success)))
                self.aruco_enable = False
                self.get_marker_pose_result = True
                break

            # Check time out for find marker stage
            if rospy.get_time() - self.last_marker_receive > 15.0:
                rospy.logerr('Get marker timeout')
                break
            r.sleep()

        # Disable aruco detect service
        set_result = self.aruco_detect_en(False)
        rospy.loginfo("Aruco detect disable result: {}".format(str(set_result.success)))
        self.aruco_enable = False
        # If get marker pose success
        if self.get_marker_pose_result:
            # Get goal pose
            self.marker_pose.pose = dock_pose_filtered.pose
            self.marker_goal_pose.pose = self.get_goal_from_marker( self.marker_pose.pose, self.x_offset, self.y_offset, self.yaw_offset )
            print("Marker pose: \n {}".format(self.marker_pose.pose))
            print("Marker Goal pose: \n {}".format(self.marker_goal_pose.pose))


            # publish marker and goal pose
            self.marker_pose.header.stamp = rospy.Time.now()
            self.marker_pose_pub.publish(self.marker_pose)

            self.marker_goal_pose.header.stamp = rospy.Time.now()
            self.marker_goal_pose_pub.publish(self.marker_goal_pose)

            # send goal to move to point action
            goal = MoveToPointGoal()
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.header.frame_id = self.base_frame
            goal.target_pose.pose = self.marker_goal_pose.pose
            self.go_to_point_client.send_goal(goal)

            # Waits for the server to finish performing the action.
            self.go_to_point_client.wait_for_result()
            # Prints out the result of executing the action
            result = self.go_to_point_client.get_result()
            print("Action move complete! {}".format(result))

            # ################################################
            # rospy.sleep(5)
            # # Lui lai
            # # send goal to move to point action
            # goal = MoveToPointGoal()
            # goal.target_pose.header.stamp = rospy.Time.now()
            # goal.target_pose.header.frame_id = self.base_frame
            # goal.target_pose.pose = Pose()
            # goal.target_pose.pose.position.x = -1.2
            # goal.target_pose.pose.orientation.w = 1
            # self.go_to_point_client.send_goal(goal)

            # # Waits for the server to finish performing the action.
            # self.go_to_point_client.wait_for_result()
            # # Prints out the result of executing the action
            # result = self.go_to_point_client.get_result()
            # print("Action move complete! {}".format(result))

            #################
            success = True
        else:
            rospy.loginfo("%s: Preempted" % self._action_name)
            self._as.set_preempted()
            success = False

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
            # publish marker goal pose
            if self.get_marker_pose_result:
                # publish marker and goal pose
                self.marker_goal_pose.header.stamp = rospy.Time.now()
                self.marker_goal_pose_pub.publish(self.marker_goal_pose)

                self.marker_pose.header.stamp = rospy.Time.now()
                self.marker_pose_pub.publish(self.marker_pose)

            r.sleep()

def parse_opts():
    from optparse import OptionParser
    parser = OptionParser()
    parser.add_option("-s", "--simulation",
                    action="store_true", dest="simulation", default=False, help="type \"-s\" if simulation")
    parser.add_option("-d", "--ros_debug",
                    action="store_true", dest="log_debug", default=False, help="log_level=rospy.DEBUG")
    parser.add_option("-c", "--config_path", dest="config_path",
                    default=os.path.join(rospkg.RosPack().get_path('go_to_marker'), 'cfg'))

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

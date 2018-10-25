#!/usr/bin/env python

import collections
import sys
import math
import time

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped

import utils

# The topic to publish control commands to
PUB_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
PUB_TOPIC_2 = '/plan_lookahead_follower/pose' # to publish plan lookahead follower to assist with troubleshooting
WINDOW_WIDTH = 5


'''
Follows a given plan using constant velocity and PID control of the steering angle
'''


class LineFollower:

    """
    Initializes the line follower
    plan: A list of length T that represents the path that the robot should follow
          Each element of the list is a 3-element numpy array of the form [x,y,theta]
    pose_topic: The topic that provides the current pose of the robot as a PoseStamped msg
    plan_lookahead: If the robot is currently closest to the i-th pose in the plan,
                    then it should navigate towards the (i+plan_lookahead)-th pose in the plan
    translation_weight: How much the error in translation should be weighted in relation
                        to the error in rotation
    rotation_weight: How much the error in rotation should be weighted in relation
                     to the error in translation
    kp: The proportional PID parameter
    ki: The integral PID parameter
    kd: The derivative PID parameter
    error_buff_length: The length of the buffer that is storing past error values
    speed: The speed at which the robot should travel
    """
    def __init__(self, plan, pose_topic, plan_lookahead, translation_weight,
                 rotation_weight, kp, ki, kd, error_buff_length, speed):
        # Store the passed parameters
        self.plan = plan
        self.plan_lookahead = plan_lookahead
        # Normalize translation and rotation weights
        self.translation_weight = translation_weight / (translation_weight+rotation_weight)
        self.rotation_weight = rotation_weight / (translation_weight+rotation_weight)
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # The error buff stores the error_buff_length most recent errors and the
        # times at which they were received. That is, each element is of the form
        # [time_stamp (seconds), error]. For more info about the data struct itself, visit
        # https://docs.python.org/2/library/collections.html#collections.deque
        self.error_buff = collections.deque(maxlen=error_buff_length)
        self.speed = speed

        print "line_follower Initialized!"
        print "plan[0]", self.plan[0]
        print "plan[plan_lookahead]", self.plan[plan_lookahead]
        print "error_buff length: ", len(self.error_buff)
        print "error_buff: ", self.error_buff

        # YOUR CODE HERE
        self.cmd_pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size=10)  # Create a publisher to PUB_TOPIC
        self.goal_pub = rospy.Publisher(PUB_TOPIC_2, PoseStamped, queue_size=10) # create a publisher for plan lookahead follower

        # Create a subscriber to pose_topic, with callback 'self.pose_cb'
        self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.pose_cb)
  
    '''
    Computes the error based on the current pose of the car
    cur_pose: The current pose of the car, represented as a numpy array [x,y,theta]
    Returns: (False, 0.0) if the end of the plan has been reached. Otherwise, returns
           (True, E) - where E is the computed error
    '''
    def compute_error(self, cur_pose):
        """
        Find the first element of the plan that is in front of the robot, and remove
        any elements that are behind the robot. To do this:
        Loop over the plan (starting at the beginning) For each configuration in the plan
            If the configuration is behind the robot, remove it from the plan
              Will want to perform a coordinate transformation to determine if
              the configuration is in front or behind the robot
            If the configuration is in front of the robot, break out of the loop
        """
        print "Computing error..."
        # check the leftmost pose in the plan pose-array and if it is behind the car then delete it
        if len(self.plan) > 0:

            left_edge = (cur_pose[2] + np.pi / 2) * 180 / 3.14 # deg
            right_edge = (cur_pose[2] - np.pi / 2) * 180 / 3.14 # deg
            angle_robot_path_point = math.atan2(cur_pose[1] - self.plan[0][1], cur_pose[0] - self.plan[0][0]) * 180 / 3.14 # deg

            # for troubleshooting if path points are not deleted correctly
            # converted angles from rad to deg for easier troubleshooting
            # print("robot position: ", cur_pose)
            # print("path point position: ", self.plan[0])
            # print("left_edge: ", left_edge)
            # print("right_edge: ", right_edge)
            # print("path point to robot vector: ",cur_pose[1] - self.plan[0][1], cur_pose[0] - self.plan[0][0])
            # print("angle of path point to robot vector",angle_robot_path_point)
            # print("path_point yaw",self.plan[0][2] * 180 / 3.14)

            behind = (angle_robot_path_point > right_edge and angle_robot_path_point < left_edge) # is path point behind robot?
            path_pose_similar_direction = (self.plan[0][2] > right_edge and self.plan[0][2] < left_edge) # is path point in similar direction as robot?
            if behind and path_pose_similar_direction and len(self.plan) > 0: # delete point if behind robot, similar direction, and not last point in path
                print "delete element: ", len(self.plan) # for troubleshooting, show path points before deleting
                self.plan.pop(0) # delete the first element in the path, since that point is behind robot and it's direction is similar to robot
                print "element deleted? : ", len(self.plan) # for troubleshooting, show path points after deleting


            PS = PoseStamped() # create a PoseStamped() msg
            PS.header.stamp = rospy.Time.now() # set header timestamp value
            PS.header.frame_id = "map" # set header frame id value
            goal_idx = min(0+self.plan_lookahead, len(self.plan)-1) # get goal index for looking ahead this many indices in the path
            PS.pose.position.x = self.plan[goal_idx][0] # set msg x position to value of the x position in the look ahead pose from the path
            PS.pose.position.y = self.plan[goal_idx][1] # set msg y position to value of the y position in the look ahead pose from the path
            PS.pose.position.z = 0 # set msg z position to 0 since robot is on the ground
            PS.pose.orientation = utils.angle_to_quaternion(self.plan[goal_idx][2]) # set msg orientation to [converted to queternion] value of the yaw angle in the look ahead pose from the path

            self.goal_pub.publish(PS) # publish look ahead follower, now you can add a Pose with topic of PUB_TOPIC_2 value in rviz

        # Check if the plan is empty. If so, return (False, 0.0)
        # YOUR CODE HERE
        if len(self.plan) == 0:
            return False, 0.0

        # At this point, we have removed configurations from the plan that are behind
        # the robot. Therefore, element 0 is the first configuration in the plan that is in
        # front of the robot. To allow the robot to have some amount of 'look ahead',
        # we choose to have the robot head towards the configuration at index 0 + self.plan_lookahead
        # We call this index the goal_index
        goal_idx = min(0+self.plan_lookahead, len(self.plan)-1)


        # Compute the translation error between the robot and the configuration at goal_idx in the plan
        # YOUR CODE HERE
        print "cur_pose: ", cur_pose
        print "lookahead pose: ", self.plan[goal_idx]
        look_ahead_position = np.array([self.plan[goal_idx][0], self.plan[goal_idx][1]]).reshape([2, 1])
        translation_robot_to_origin = np.array([ -cur_pose[0], -cur_pose[1]]).reshape([2, 1])
        look_ahead_position_translated = look_ahead_position + translation_robot_to_origin
        rotation_matrix_robot_to_x_axis = utils.rotation_matrix(-cur_pose[2])
        look_ahead_position_translated_and_rotated = rotation_matrix_robot_to_x_axis * look_ahead_position_translated
        print "look_ahead_position_translated_and_rotated: ", look_ahead_position_translated_and_rotated
        x_error = look_ahead_position_translated_and_rotated[0][0] # This is the distance that the robot is behind the lookahead point parallel to the path
        y_error = look_ahead_position_translated_and_rotated[1][0] # This is the distance away from the path, perpendicular from the path to the robot
        translation_error = -1 * math.tan(y_error / x_error) * math.pi / 180 # angle in rad to drive along hypotenuse toward the look ahead point
        translation_error *= y_error/x_error # make the robot turn more sharply if far away from path

        # translation_error = np.sqrt(np.square(cur_pose[0] - self.plan[goal_idx][0]) + np.square(cur_pose[1] - self.plan[goal_idx][1]))

        print "Translation error: ", translation_error

        # Compute the total error
        # Translation error was computed above
        # Rotation error is the difference in yaw between the robot and goal configuration
        #   Be careful about the sign of the rotation error
        # YOUR CODE HERE
        rotation_error = cur_pose[2] - self.plan[goal_idx][2]
        print "Rotation error: ", rotation_error

        error = self.translation_weight * translation_error + self.rotation_weight * rotation_error
        print "Overall error: ", error

        return True, error
    
    '''
    Uses a PID control policy to generate a steering angle from the passed error
    error: The current error
    Returns: The steering angle that should be executed
    '''
    def compute_steering_angle(self, error):
        print "Computing steering angle..."
        now = rospy.Time.now().to_sec()  # Get the current time

        # Compute the derivative error using the passed error, the current time,
        # the most recent error stored in self.error_buff, and the most recent time
        # stored in self.error_buff
        # YOUR CODE HERE

        deriv_error = 0  # for the first iteration, this is true
        integ_error = 0
        print "setting deriv and integ error to 0"
        print "error_buff len", len(self.error_buff)
        if len(self.error_buff) > 0:
            time_delta = now - self.error_buff[-1][1]       # -1 means peeking the rightmost element (most recent)
            error_delta = error - self.error_buff[-1][0]

            deriv_error = error_delta / time_delta
            print "computed deriv error: ", deriv_error

        # Add the current error to the buffer
        self.error_buff.append((error, now))

        # Compute the integral error by applying rectangular integration to the elements
        # of self.error_buff:
        # ://chemicalstatistician.wordpress.com/2014/01/20/rectangular-integration-a-k-a-the-midpoint-rule/
        # YOUR CODE HERE
        error_array = []
        if len(self.error_buff) > 0:
            for err in self.error_buff:
                error_array.append(err[0])
            integ_error = np.trapz(error_array)
            print "computed integ error: ", integ_error

        # Compute the steering angle as the sum of the pid errors
        # YOUR CODE HERE
        return -(self.kp*error + self.ki*integ_error + self.kd * deriv_error)
    
    '''
    Callback for the current pose of the car
    msg: A PoseStamped representing the current pose of the car
    This is the exact callback that we used in our solution, but feel free to change it
    '''
    def pose_cb(self, msg):
        print ""
        time.sleep(0)
        print "Callback received current pose. "
        cur_pose = np.array([msg.pose.position.x,
                             msg.pose.position.y,
                             utils.quaternion_to_angle(msg.pose.orientation)])
        print "Current pose: ", cur_pose

        success, error = self.compute_error(cur_pose)
        print "Success, Error: ", success, error

        if not success:
            # We have reached our goal
            self.pose_sub = None  # Kill the subscriber
            self.speed = 0.0  # Set speed to zero so car stops

        delta = self.compute_steering_angle(error)

        print "delta is %f" % delta

        # Setup the control message
        ads = AckermannDriveStamped()
        ads.header.frame_id = '/map'
        ads.header.stamp = rospy.Time.now()
        ads.drive.steering_angle = delta
        ads.drive.speed = self.speed

        # Send the control message
        self.cmd_pub.publish(ads)


def main():

    rospy.init_node('line_follower', anonymous=True)  # Initialize the node
    """
    Load these parameters from launch file
    We provide suggested starting values of params, but you should
    tune them to get the best performance for your system
    Look at constructor of LineFollower class for description of each var
    'Default' values are ones that probably don't need to be changed (but you could for fun)
    'Starting' values are ones you should consider tuning for your system
    """
    # YOUR CODE HERE
    plan_topic = rospy.get_param('~plan_topic')  # Default val: '/planner_node/car_plan'
    pose_topic = rospy.get_param('~pose_topic')  # Default val: '/sim_car_pose/pose'
    plan_lookahead = rospy.get_param('~plan_lookahead')  # Starting val: 5
    translation_weight = rospy.get_param('~translation_weight')  # Starting val: 1.0
    rotation_weight = rospy.get_param('~rotation_weight')  # Starting val: 0.0
    kp = rospy.get_param('~kp')  # Startinig val: 1.0
    ki = rospy.get_param('~ki')  # Starting val: 0.0
    kd = rospy.get_param('~kd')  # Starting val: 0.0
    error_buff_length = rospy.get_param('~error_buff_length')  # Starting val: 10
    speed = rospy.get_param('~speed')  # Default val: 1.0

    raw_input("Press Enter to when plan available...")  # Waits for ENTER key press

    # Use rospy.wait_for_message to get the plan msg
    # Convert the plan msg to a list of 3-element numpy arrays
    #     Each array is of the form [x,y,theta]
    # Create a LineFollower object

    raw_plan = rospy.wait_for_message(plan_topic, PoseArray)

    # raw_plan is a PoseArray which has an array of geometry_msgs/Pose called poses

    plan_array = []

    for pose in raw_plan.poses:
        plan_array.append(np.array([pose.position.x, pose.position.y, utils.quaternion_to_angle(pose.orientation)]))

    print "Len of plan array: %d" % len(plan_array)
    # print plan_array

    try:
        if raw_plan:
            pass
    except rospy.ROSException:
        exit(1)

    lf = LineFollower(plan_array, pose_topic, plan_lookahead, translation_weight,
                      rotation_weight, kp, ki, kd, error_buff_length, speed)  # Create a Line follower

    rospy.spin()  # Prevents node from shutting down


if __name__ == '__main__':
    main()

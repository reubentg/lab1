#!/usr/bin/env python

import rospy
import numpy as np
import math
import sys

import utils

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

SCAN_TOPIC = '/scan'  # The topic to subscribe to for laser scans
CMD_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'  # The topic to publish controls to
POSE_TOPIC = '/sim_car_pose/pose'   # The topic to subscribe to for current pose of the car
# NOTE THAT THIS IS ONLY NECESSARY FOR VIZUALIZATION

VIZ_TOPIC = '/laser_wanderer/rollouts'  # The topic to publish to for vizualizing
# the computed rollouts. Publish a PoseArray.

MAX_PENALTY = 10000  # The penalty to apply when a configuration in a rollout
# goes beyond the corresponding laser scan
                    

'''
Wanders around using minimum (steering angle) control effort while avoiding crashing
based off of laser scans. 
'''


class LaserWanderer:

    """
    Initializes the LaserWanderer
    rollouts: An NxTx3 numpy array that contains N rolled out trajectories, each
              containing T poses. For each trajectory, the t-th element represents
              the [x,y,theta] pose of the car at time t+1
    deltas: An N dimensional array containing the possible steering angles. The n-th
            element of this array is the steering angle that would result in the
            n-th trajectory in rollouts
    speed: The speed at which the car should travel
    compute_time: The amount of time (in seconds) we can spend computing the cost
    laser_offset: How much to shorten the laser measurements
    """
    def __init__(self, rollouts, deltas, speed, compute_time, laser_offset):
        # Store the params for later
        self.rollouts = rollouts
        self.deltas = deltas
        self.speed = speed
        self.compute_time = compute_time
        self.laser_offset = laser_offset
        self.show_all_poses = False # if true all poses will be vizualized, if false only final pose

        # TODO: Double check the fields below
        self.cmd_pub = rospy.Publisher(CMD_TOPIC, AckermannDriveStamped, queue_size=10)

        # Create a subscriber to laser scans that uses the self.wander_cb callback
        self.laser_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self.wander_cb)

        # Create a publisher for vizualizing trajectories. Will publish PoseArrays
        self.viz_pub = rospy.Publisher(VIZ_TOPIC, PoseArray)

        # Create a subscriber to the current position of the car
        self.viz_sub = rospy.Subscriber(POSE_TOPIC, PoseStamped, self.viz_sub_cb)
        # NOTE THAT THIS VIZUALIZATION WILL ONLY WORK IN SIMULATION. Why?
    
    '''
    Vizualize the rollouts. Transforms the rollouts to be in the frame of the world.
    Only display the last pose of each rollout to prevent lagginess
    msg: A PoseStamped representing the current pose of the car
    '''
    def viz_sub_cb(self, msg):
        # Create the PoseArray to publish. Will contain N poses, where the n-th pose
        # represents the last pose in the n-th trajectory
        # print "inside viz_sub_cb"

        cur_pose = np.array([msg.pose.position.x,
                             msg.pose.position.y,
                             utils.quaternion_to_angle(msg.pose.orientation)]) # current car pose

        if self.show_all_poses: # all poses in each rollout
            pose_range = range(0, self.rollouts.shape[1])
        else: # only final pose in each rollout
            pose_range = range(self.rollouts.shape[1] - 2, self.rollouts.shape[1])

        # Read http://docs.ros.org/jade/api/geometry_msgs/html/msg/PoseArray.html
        PA = PoseArray() # create a PoseArray() msg
        PA.header.stamp = rospy.Time.now() # set header timestamp value
        PA.header.frame_id = "map" # set header frame id value
        PA.poses = []

        for i in range(0, self.rollouts.shape[0]): # for each [7] rollouts
            for j in pose_range: # for pose in range(0, 300) to show all, or range(299,300) to show only final pose
                P = Pose()
                pose = self.rollouts[i, j, :] # self.rollouts[i, 299, :] will be an array of [x, y, theta] for the final pose (last j index) for rollout i

                # This is in car frame, so assumes straight is x axis
                # P.position.x = pose[0]
                # P.position.y = pose[1]
                # P.position.z = 0
                # P.orientation = utils.angle_to_quaternion(pose[2])

                # Transform pose from car frame to map frame

                # Method 1: Map Frame to Robot Frame
                # Rotation must be done before translation with this method, but it's ok to do both in one step
                # First find translation matrix [2, 1] from map origin to robot position
                # Second find rotation matrix [2, 2] from map x axis to robot x axis
                # Third rotate the rollout position about the origin of the robot axis [a.k.a. robot position] with the same angle as from map x axis to robot x axis
                # Fourth translate the rollout position with the same translation matrix as from the map origin to the robot position
                translation_map_to_robot = np.array([[cur_pose[0]], [cur_pose[1]]]).reshape([2, 1])
                rotation_matrix_map_to_robot = utils.rotation_matrix(cur_pose[2])
                rollout_position = np.array([[pose[0]], [pose[1]]]).reshape([2, 1])
                map_position = rotation_matrix_map_to_robot * rollout_position + translation_map_to_robot
                P.position.x = map_position[0]
                P.position.y = map_position[1]
                P.position.z = 0
                P.orientation = utils.angle_to_quaternion(pose[2] + cur_pose[2]) # car's yaw angle + rollout pose's angle from car

                PA.poses.append(P)
        # print "Publishing Rollout Vizualization"
        self.viz_pub.publish(PA)

    '''
    Compute the cost of one step in the trajectory. It should penalize the magnitude
    of the steering angle. It should also heavily penalize crashing into an object
    (as determined by the laser scans)
    delta: The steering angle that corresponds to this trajectory
    rollout_pose: The pose in the trajectory 
    laser_msg: The most recent laser scan
    '''
    def compute_cost(self, delta, rollout_pose, laser_msg):
        """
        Initialize the cost to be the magnitude of delta
        Consider the line that goes from the robot to the rollout pose
        Compute the angle of this line with respect to the robot's x axis
        Find the laser ray that corresponds to this angle
        Add MAX_PENALTY to the cost if the distance from the robot to the rollout_pose
        is greater than the laser ray measurement - np.abs(self.laser_offset)
        Return the resulting cost
        Things to think about:
          What if the angle of the pose is less (or greater) than the angle of the
          minimum (or maximum) laser scan angle
          What if the corresponding laser measurement is NAN?
        NOTE THAT NO COORDINATE TRANSFORMS ARE NECESSARY INSIDE OF THIS FUNCTION
        """
        # YOUR CODE HERE
        cost = abs(delta)
        angle = math.atan2(rollout_pose[1], rollout_pose[0])
        laser_ray_index = int(round((angle - laser_msg.angle_min) / laser_msg.angle_increment))
        pose_dist = math.pow(rollout_pose[0], 2) + math.pow(rollout_pose[1], 2)
        laser_dist = laser_msg.ranges[laser_ray_index]


        if math.isnan(laser_dist) or laser_dist == 0.0:
            laser_dist = np.Inf
        # if delta == -0.34:
        #     print "Ranges: %.2f" % laser_dist,
        # elif delta == 0.34:
        #     print "%.2f" % laser_dist
        #     print ", pose_dist: ", pose_dist
        #     print ""
        # else:
        #     print "%.2f" % laser_dist,

        if laser_dist - np.abs(self.laser_offset) < pose_dist:
            cost += MAX_PENALTY
        return cost

    '''
    Controls the steering angle in response to the received laser scan. Uses approximately
    self.compute_time amount of time to compute the control
    msg: A LaserScan
    '''
    def wander_cb(self, msg):
        # print 'Inside wander_cb'
        # print "LaserScan MSG: ", type(msg)
        # print "msg.ranges", type(np.array(msg.ranges)), np.array(msg.ranges).shape
        # rosmsg show sensor_msgs/LaserScan
        # std_msgs/Header header
        #   uint32 seq
        #   time stamp
        #   string frame_id
        # float32 angle_min
        # float32 angle_max
        # float32 angle_increment
        # float32 time_increment
        # float32 scan_time
        # float32 range_min
        # float32 range_max
        # float32[] ranges
        # float32[] intensities


        start = rospy.Time.now().to_sec()  # Get the time at which this function started

        # A N dimensional matrix that should be populated with the costs of each
        # trajectory up to time t <= T
        delta_costs = np.zeros(self.deltas.shape[0], dtype=np.float) # array([ 0.,  0.,  0.,  0.,  0.,  0.,  0.])
        traj_depth = 0

        # Evaluate the cost of each trajectory. Each iteration of the loop should calculate
        # the cost of each trajectory at time t = traj_depth and add those costs to delta_costs
        # as appropriate

        # Pseudo code
        # while(you haven't run out of time AND traj_depth < T):
        #   for each trajectory n:
        #       delta_costs[n] += cost of the t=traj_depth step of trajectory n
        #   traj_depth += 1
        # YOUR CODE HERE
        while rospy.Time.now().to_sec() - start < self.compute_time and traj_depth < self.rollouts.shape[1]:
            # print "While loop in wander_cb w: ", (rospy.Time.now().to_sec() - start)
            for i in range(0, self.rollouts.shape[0]): # for each [7] rollouts
                # print "Compute cost for rollout #", i, ", traj_depth #", traj_depth
                delta_costs[i] += self.compute_cost(self.deltas[i], self.rollouts[i, traj_depth, :], msg)
              #  print "computed cost for traj depth# ", traj_depth, " rollout# " , i, ", cost sum # " , delta_costs[1]
            traj_depth += 1


        # Find the delta that has the smallest cost and execute it by publishing
        # YOUR CODE HERE
        ind_mid_minus_1 = math.floor(delta_costs.shape[0] / 2)
        ind_mid = ind_mid_minus_1 + 1
        ind_mid_plus_1 = ind_mid +1

        # max_front_delta_cost = max(
        #     delta_costs[ind_mid_minus_1],
        #     delta_costs[ind_mid],
        #     delta_costs[ind_mid_plus_1]
        # )
        # delta_costs[ind_mid_minus_1] = max_front_delta_cost
        # delta_costs[ind_mid] = max_front_delta_cost
        # delta_costs[ind_mid_plus_1] = max_front_delta_cost

        min_delta_cost_index = np.argmin(delta_costs)
        delta = self.deltas[min_delta_cost_index]

        # print "chosen rollout index: %d" % min_delta_cost_index
        

        # Setup the control message
        ads = AckermannDriveStamped()
        ads.header.frame_id = '/map'
        ads.header.stamp = rospy.Time.now()
        ads.drive.steering_angle = delta
        ads.drive.speed = self.speed
        self.cmd_pub.publish(ads)
        np.set_printoptions(precision=3)
        print "costs:", delta_costs


'''
Apply the kinematic model to the passed pose and control
  pose: The current state of the robot [x, y, theta]
  control: The controls to be applied [v, delta, dt]
  car_length: The length of the car
Returns the resulting pose of the robot
'''


def kinematic_model_step(pose, control, car_length):
    # Apply the kinematic model
    # Make sure your resulting theta is between 0 and 2*pi
    # Consider the case where delta == 0.0

    x = pose[0]
    y = pose[1]
    theta = pose[2]
    q = np.matrix([x], [y])           #robot posture in base frame
    R = utils.rotation_matrix(theta)

    # pose_translated = q[]*R[]



    # YOUR CODE HERE
    pass
    

'''
Repeatedly apply the kinematic model to produce a trajectory for the car
  init_pose: The initial pose of the robot [x,y,theta]
  controls: A Tx3 numpy matrix where each row is of the form [v,delta,dt]
  car_length: The length of the car
Returns a Tx3 matrix where the t-th row corresponds to the robot's pose at time t+1
'''


def generate_rollout(init_pose, controls, car_length):
    # init_pose: array([ 0.,  0.,  0.]) where [x_t-1, y_t-1, theta_t-1]
    # controls: controls.shape (300, 3): 300 controls, each control has [v, delta, dt]
    # car_length: 0.33
    # returns a [300, 3] rollout: 300 poses with [x, y, theta]

    # print "controls", type(controls), controls.shape
    # print "controls[1][0]", type(controls[1][0]), controls[1][0] # TODO: find out what the difference is
    # print "controls[1,0]", type(controls[1,0]), controls[1,0]

    # use initial pose as the previous pose in the first loop iteration
    theta_t_minus_1 = init_pose[2]
    xt_minus_1 = init_pose[0]
    yt_minus_1 = init_pose[1]
    # create array to hold rollout result
    rollout = np.zeros([300,3])
    for i in xrange(300):
        v = controls[i, 0]
        delta = controls[i, 1]
        dt = controls[i, 2]
        beta = math.atan((1.0 / 2.0) * math.tan(delta))
        theta = theta_t_minus_1 + v / car_length * math.sin(2 * beta * dt)
        if beta == 0 and theta == 0:
            xt = xt_minus_1 + v * dt # distance = speed * time
            yt = 0 # no change in y because we are going straight along the x axis in the car's frame
        else:
            xt = xt_minus_1 + car_length / math.sin(2 * beta) * (math.sin(theta) - math.sin(theta_t_minus_1))
            # print "xt,", xt
            yt = yt_minus_1 + car_length / math.sin(2 * beta) * (-math.cos(theta) + math.cos(theta_t_minus_1))
            # print "yt", yt
        # use current pose as the previous pose in the next loop iteration
        theta_t_minus_1 = theta
        xt_minus_1 = xt
        yt_minus_1 = yt

        # save rollout pose i
        rollout[i,:] = [xt, yt, theta]
    return rollout


'''
Helper function to generate a number of kinematic car rollouts
    speed: The speed at which the car should travel
    min_delta: The minimum allowed steering angle (radians)
    max_delta: The maximum allowed steering angle (radians)
    delta_incr: The difference (in radians) between subsequent possible steering angles
    dt: The amount of time to apply a control for
    T: The number of time steps to rollout for
    car_length: The length of the car
Returns a NxTx3 numpy array that contains N rolled out trajectories, each
containing T poses. For each trajectory, the t-th element represents the [x,y,theta]
pose of the car at time t+1
'''


def generate_mpc_rollouts(speed, min_delta, max_delta, delta_incr, dt, T, car_length):

    deltas = np.arange(min_delta, max_delta, delta_incr) # array([-0.34, -0.22666667, -0.11333333,  0.,  0.11333333, 0.22666667,  0.34      ])
    # array([-0.34, -0.22666667, -0.11333333,  0., 0, 0.,  0.11333333, 0.22666667,  0.34 ])
    # add more front rollouts and then two front rollouts will be moved to car's edges
    # and then the min distance will be used from all front rollouts to represent the forward laser ray
    deltas = np.concatenate((
        deltas[0:deltas.shape[0] / 2], deltas[(deltas.shape[0] / 2)], deltas[(deltas.shape[0] / 2)],
        deltas[(deltas.shape[0] / 2)], deltas[(deltas.shape[0] + 1) / 2:deltas.shape[0]]), axis=None)

    N = deltas.shape[0] # 7 for sim, 9 for robot

    init_pose = np.array([0.0, 0.0, 0.0], dtype=np.float) # array([ 0.,  0.,  0.])

    rollouts = np.zeros((N, T, 3), dtype=np.float) # rollout.shape (7, 300, 3): 7 rollouts, 300 poses each, each pose has [x, y, theta]
    for i in xrange(N): # for each rollout [yet-to-be-created]
        controls = np.zeros((T, 3), dtype=np.float) # controls.shape (300, 3): 300 controls, each control has [v, delta, dt]
        controls[:, 0] = speed # velocity as a number, not vector
        controls[:, 1] = deltas[i] # delta for each rollout
        controls[:, 2] = dt # The amount of time to apply a control for
        rollouts[i, :, :] = generate_rollout(init_pose, controls, car_length) # create rollout

        # shift only the middle rollouts to the car edges
        if i >= math.floor(N / 2)-1 and i <= math.floor(N / 2):
            for depth in range(rollouts[i, :, :].shape[0]):
                rollouts[i, depth, :][1] -= car_length *2
        if i >= math.ceil(N / 2) and i <= math.ceil(N / 2) +1:
            for depth in range(rollouts[i, :, :].shape[0]):
                rollouts[i, depth, :][1] += car_length *2

        # ind_mid_minus_1 = math.floor(N / 2)
        # ind_mid = ind_mid_minus_1 + 1
        # ind_mid_plus_1 = ind_mid + 1
        # if i == ind_mid_minus_1:
        #     for depth in range(rollouts[i, :, :].shape[0]):
        #         rollouts[i, depth, :][1] -= car_length
        # # elif i == ind_mid:
        # elif i == ind_mid_plus_1:
        #     for depth in range(rollouts[i, :, :].shape[0]):
        #         rollouts[i, depth, :][1] += car_length



    return rollouts, deltas


def main():
    print "Laser Wanderer Running!"
    rospy.init_node('laser_wanderer', anonymous=True)

    # Load these parameters from launch file
    # We provide suggested starting values of params, but you should
    # tune them to get the best performance for your system
    # Look at constructor of LaserWanderer class for description of each var
    # 'Default' values are ones that probably don't need to be changed (but you could for fun)
    # 'Starting' values are ones you should consider tuning for your system
    # YOUR CODE HERE
    speed = rospy.get_param('~speed')  # Default val: 1.0
    min_delta = rospy.get_param('~min_delta')  # Default val: -0.34
    max_delta = rospy.get_param('~max_delta')  # Default val: 0.341
    delta_incr = rospy.get_param('~delta_incr')  # Starting val: 0.34/3 (consider changing the denominator)
    dt = rospy.get_param('~dt')  # Default val: 0.01
    T = rospy.get_param('~T')  # Starting val: 300
    compute_time = rospy.get_param('~compute_time')  # Default val: 0.09
    laser_offset = rospy.get_param('~laser_offset')  # Starting val: 1.0

    # DO NOT ADD THIS TO YOUR LAUNCH FILE, car_length is already provided by teleop.launch
    car_length = rospy.get_param("car_kinematics/car_length", 0.33)

    print "Generating Rollouts"
    # Generate the rollouts
    rollouts, deltas = generate_mpc_rollouts(speed, min_delta, max_delta, delta_incr, dt, T, car_length)

    print "Constructing Laser Wanderer"
    # Create the LaserWanderer
    lw = LaserWanderer(rollouts, deltas, speed, compute_time, laser_offset)

    # Keep the node alive
    rospy.spin()
  

if __name__ == '__main__':
    main()

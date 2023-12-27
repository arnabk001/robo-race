#!/usr/bin/env python

import collections
import sys

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped

import matplotlib.pyplot as plt

import utils

# The topic to publish control commands to
PUB_TOPIC = '/car/mux/ackermann_cmd_mux/input/navigation'
ERROR_LIST = []

'''
Follows a given plan using constant velocity and PID control of the steering angle
'''
class LineFollower:

  '''
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
  '''
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
    time_now = rospy.Time.now().to_sec() # Get the current time
    self.error_buff.append((0.0, time_now))
    self.speed = speed
    
    # YOUR CODE HERE
    self.cmd_pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size = 1)# Create a publisher to PUB_TOPIC
    self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.pose_cb) # Create a subscriber to pose_topic, with callback 'self.pose_cb'
  
  '''
  Computes the error based on the current pose of the car
    cur_pose: The current pose of the car, represented as a numpy array [x,y,theta]
  Returns: (False, 0.0) if the end of the plan has been reached. Otherwise, returns
           (True, E) - where E is the computed error
  '''
  def compute_error(self, cur_pose):
    
    # Find the first element of the plan that is in front of the robot, and remove
    # any elements that are behind the robot. To do this:
    # Loop over the plan (starting at the beginning) For each configuration in the plan
        # If the configuration is behind the robot, remove it from the plan
        #   Will want to perform a coordinate transformation to determine if 
        #   the configuration is in front or behind the robot
        # If the configuration is in front of the robot, break out of the loop
    while len(self.plan) > 0:
      # YOUR CODE HERE
      # calculate the angle of the distance
      # print("entered the loop - path check, plan length",len(self.plan))
      for index, plan_item in enumerate(self.plan):
        # x2, y2 = plan_item[:2]
        # x1, y1, theta1 = cur_pose[:]
        # # check if the angles are in deg or rad, process accordingly
        # # assuming both in radians
        # theta2 = np.arctan2((y2-y1), (x2-x1))

        # if np.absolute(theta2-theta1)>(np.pi/2):
        #   self.plan.pop(index)
        # else:
        #   break

        xref, yref, thetaref = cur_pose
        # print("cur_pose:", xref, yref, thetaref)
        x, y, theta = plan_item
        # print("plan_pose:", x, y, theta)
        e_at = (x - xref)*np.cos(thetaref) + (y - yref)*np.sin(thetaref)
        # print("e_at:", e_at)

        if e_at > 0:
            # return True  # point2 is in front of point1
            break
        else:
            # return False  # point2 is not in front of point1
            self.plan.pop(index)
      else:
        continue
      break
      # pass
            
    # print("exited the loop")
    # Check if the plan is empty. If so, return (False, 0.0)
    # YOUR CODE HERE
    if len(self.plan) == 0:
      return (False, 0.0)
    
    # At this point, we have removed configurations from the plan that are behind
    # the robot. Therefore, element 0 is the first configuration in the plan that is in 
    # front of the robot. To allow the robot to have some amount of 'look ahead',
    # we choose to have the robot head towards the configuration at index 0 + self.plan_lookahead
    # We call this index the goal_index
    goal_idx = min(0+self.plan_lookahead, len(self.plan)-1)
   
    # Compute the translation error between the robot and the configuration at goal_idx in the plan
    # YOUR CODE HERE
    # translation_error = np.sqrt(np.sum((cur_pose[:2]-self.plan[goal_idx][:2])**2))
    xref, yref, thetaref = cur_pose
    x, y, theta = self.plan[goal_idx]

    e_ct = -(x - xref)*np.sin(thetaref) + (y - yref)*np.cos(thetaref)
    translation_error = e_ct
    rotational_error = np.absolute(self.plan[goal_idx][2] - cur_pose[2])
    # Compute the total error
    # Translation error was computed above
    # Rotation error is the difference in yaw between the robot and goal configuration
    #   Be carefult about the sign of the rotation error
    # YOUR CODE HERE
    error = self.translation_weight * translation_error + self.rotation_weight * rotational_error
    
    return True, error
    
    
  '''
  Uses a PID control policy to generate a steering angle from the passed error
    error: The current error
  Returns: The steering angle that should be executed
  '''
  def compute_steering_angle(self, error):
    now = rospy.Time.now().to_sec() # Get the current time
    
    # Compute the derivative error using the passed error, the current time,
    # the most recent error stored in self.error_buff, and the most recent time
    # stored in self.error_buff
    # YOUR CODE HERE
    deriv_error = (error - self.error_buff[-1][0])/(now - self.error_buff[-1][1])
    
    # Add the current error to the buffer
    self.error_buff.append((error, now))
    ERROR_LIST.append([error, now])
    # plt.plot(error, now)
    # plt.title("error vs time plot")
    # # plt.draw()
    # plt.pause(0.1)


    # Compute the integral error by applying rectangular integration to the elements
    # of self.error_buff: https://chemicalstatistician.wordpress.com/2014/01/20/rectangular-integration-a-k-a-the-midpoint-rule/
    # YOUR CODE HERE
    integ_error = 0
    for i in range(1, len(self.error_buff)):
      
      dt = (self.error_buff[i][1] - self.error_buff[i-1][1])
      y = 0.5*(self.error_buff[i][0] + self.error_buff[i-1][0])
      integ_error += y*dt

    # Compute the steering angle as the sum of the pid errors
    # YOUR CODE HERE
    return self.kp*error + self.ki*integ_error + self.kd * deriv_error
    
  '''
  Callback for the current pose of the car
    msg: A PoseStamped representing the current pose of the car
    This is the exact callback that we used in our solution, but feel free to change it
  '''  
  def pose_cb(self, msg):
    cur_pose = np.array([msg.pose.position.x,
                         msg.pose.position.y,
                         utils.quaternion_to_angle(msg.pose.orientation)])
    success, error = self.compute_error(cur_pose)
    
    if not success:
      # We have reached our goal
      self.pose_sub = None # Kill the subscriber
      self.speed = 0.0 # Set speed to zero so car stops
      
    delta = self.compute_steering_angle(error)
    
    # file1 = open("home\robot\mushr_ws\src\assignment3\test1.txt", "a")  # append mode
    # file1.write(str(error)+"\n")
    # file1.close()

    # Setup the control message
    ads = AckermannDriveStamped()
    ads.header.frame_id = '/map'
    ads.header.stamp = rospy.Time.now()
    ads.drive.steering_angle = delta
    ads.drive.speed = self.speed
    
    # Send the control message
    self.cmd_pub.publish(ads)

def main():

  rospy.init_node('line_follower', anonymous=True) # Initialize the node
  
  # Load these parameters from launch file
  # We provide suggested starting values of params, but you should
  # tune them to get the best performance for your system
  # Look at constructor of LineFollower class for description of each var
  # 'Default' values are ones that probably don't need to be changed (but you could for fun)
  # 'Starting' values are ones you should consider tuning for your system
  # YOUR CODE HERE
  plan_topic = str(rospy.get_param("~plan_topic", None))
  pose_topic = str(rospy.get_param("~pose_topic", None))
  plan_lookahead =int(rospy.get_param("~plan_lookahead", None))
  translation_weight = float(rospy.get_param("~translation_weight", None))
  rotation_weight = float(rospy.get_param("~rotation_weight", None))
  kp = float(rospy.get_param("~kp", None))
  ki = float(rospy.get_param("~ki", None))
  kd = float(rospy.get_param("~kd", None))
  error_buff_length = float(rospy.get_param("~error_buff_length", None))
  speed = float(rospy.get_param("~speed", None))

  # plan_topic ='/planner_node/car_plan' # Default val: '/planner_node/car_plan'
  # pose_topic = '/car/pose' # Default val: '/car/pose'
  # plan_lookahead = 5 # Starting val: 5
  # translation_weight = 1.0 # Starting val: 1.0
  # rotation_weight = 0.0 # Starting val: 0.0
  # kp = 1.0 # Startinig val: 1.0
  # ki = 0.0 # Starting val: 0.0
  # kd = 0.0 # Starting val: 0.0
  # error_buff_length = 10 # Starting val: 10
  # speed = 1.0 # Default val: 1.0

  raw_input("Press Enter to when plan available...")  # Waits for ENTER key press
  
  # Use rospy.wait_for_message to get the plan msg
  # Convert the plan msg to a list of 3-element numpy arrays
  #     Each array is of the form [x,y,theta]
  # Create a LineFollower object
  # YOUR CODE HERE

  data = rospy.wait_for_message(plan_topic, PoseArray, timeout=10)
  
  # print("The plan topic message length:", len(data))
  
  
  plan = []
  for item in data.poses:
    item_list = [item.position.x,item.position.y,utils.quaternion_to_angle(item.orientation)]
    plan.append(item_list)
  
  # print("plan length:",len(plan))

  LineFollowerObj = LineFollower(plan, pose_topic, plan_lookahead, translation_weight,
                                 rotation_weight, kp, ki, kd, error_buff_length, speed)

  rospy.spin() # Prevents node from shutting down

if __name__ == '__main__':
  main()
  ERROR_LIST = np.array(ERROR_LIST)
  plt.plot(range(len(ERROR_LIST)), ERROR_LIST[:,0])
  np.save('/home/robot/mushr_ws/src/assignment3/test3.npy', ERROR_LIST)
  plt.show()

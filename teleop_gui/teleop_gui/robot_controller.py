# Python math library
import math 
 
# ROS client library for Python
import rclpy 
 
# Enables pauses in the execution of code
from time import sleep 
 
# Used to create nodes
from rclpy.node import Node
 
# Enables the use of the string message type
# from std_msgs.msg import String 
 
# Twist is linear and angular velocity
from geometry_msgs.msg import Twist 
# from geometry_msgs.msg import TwistStamped
                     
# Position, orientation, linear velocity, angular velocity
from nav_msgs.msg import Odometry

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from PIL import Image as pil_image
from PIL import ImageTk

# Handles LaserScan messages to sense distance to obstacles (i.e. walls)        
from sensor_msgs.msg import LaserScan    

# Handles quality of service for LaserScan data
from rclpy.qos import qos_profile_sensor_data 

# Handle Pose messages
from geometry_msgs.msg import Pose 
 
# Handle float64 arrays
from std_msgs.msg import Float64MultiArray
 
# Scientific computing library
import numpy as np 

# Create a class derived from ROS2 Node class
class Controller(Node):
  # Initialize the Node
  def __init__(self):
    super().__init__('Controller')

    self.bManualControl = False

    self.nCameraResX = 640
    self.nCameraResY = 480

    self.bridge = CvBridge()
    self.cv_image = pil_image.new('RGB', (self.nCameraResX, self.nCameraResY))

    self.arrLidarRanges = []
    # Subscribe to messages of type nav_msgs/Odometry 
    # that provides position and orientation of the robot
    # Nice to have, but NOT required to control our robot
    self.odom_subscriber = self.create_subscription(
                           Odometry,
                           '/odom',
                           self.odom_callback,
                           10)

    # Subscribe to camera data
    self.subscription = self.create_subscription(
        Image,
        '/camera/image_raw',
        self.camera_callback,
        qos_profile_sensor_data)   
    self.subscription  # prevent unused variable warning                            

    # Create a publisher of the estimated position (x, y, yaw)  
    # if we want the robot's state info to be published. 
    # Note: this is for other nodes to see
    # the state information from outside. It is NOT required 
    # for our demo to work. All it does is repackaging the robot
    # state information and republishing it under a different topic.
    # The type of message is std_msgs/Float64MultiArray
    # self.publisher_state_est = self.create_publisher(
    #                            Float64MultiArray, 
    #                            '/info/state_est', 
    #                            10)                           

    # Create a subscriber 
    # This node subscribes to messages of type 
    # geometry_msgs/Twist.msg. We are listening to the velocity commands here.
    # The maximum number of queued messages is 10.
  #    self.velocity_subscriber = self.create_subscription(
  #                               Twist, # TwistStamped, #Twist,
  #                               '/demo/cmd_vel', #'/cmd_vel_unstamped', #'/demo/cmd_vel',
  #                               self.velocity_callback,
  #                               10)                           

    # Create a subscriber
    # This node subscribes to messages of type Float64MultiArray  
    # over a topic named: /demo/state_est
    # The message represents the current estimated state:
    #   [x, y, yaw]
    # The callback function is called as soon as a message 
    # is received.
    # The maximum number of queued messages is 10.
    #self.subscription = self.create_subscription(
    #                    Float64MultiArray,
    #                    '/demo/state_est',
    #                    self.state_estimate_callback,
    #                    10)
    #self.subscription  # prevent unused variable warning
 
    # Subscribe to messages of type sensor_msgs/LaserScan
    self.scan_subscriber = self.create_subscription(
                           LaserScan,
                           '/laser_controller/out',
                           self.scan_callback,
                           qos_profile=qos_profile_sensor_data)
                            
    # Publish the desired linear and angular velocity of the robot (in the
    # robot chassis coordinate frame) to the /cmd_vel_unstamped topic. 
    # The diff_drive controller that our robot uses will read this topic 
    # and move the robot accordingly.
    self.publisher_ = self.create_publisher(
                      Twist, #TwistStamped, 
                      '/diff_cont/cmd_vel_unstamped', # For ROS2 diff. drive controller
                      #"/cmd_vel",  # For Gazebo diff. drive controller
                      10)
 
    # Initialize the LaserScan sensor readings to some large value
    # Values are in meters.
    self.left_dist = 999999.9 # Left
    self.leftfront_dist = 999999.9 # Left-front
    self.front_dist = 999999.9 # Front
    self.rightfront_dist = 999999.9 # Right-front
    self.right_dist = 999999.9 # Right
 
    ################### ROBOT CONTROL PARAMETERS ##################
    # Maximum forward speed of the robot in meters per second
    # Any faster than this and the robot risks falling over.
    self.forward_speed = 0.5
 
    # Current position and orientation of the robot in the global 
    # reference frame
    self.current_x = 0.0
    self.current_y = 0.0
    self.current_z = 0.0
    
    self.current_roll = 0.0
    self.current_pitch = 0.0
    self.current_yaw = 0.0
 
    ############# WALL FOLLOWING PARAMETERS #######################     
    # Finite states for the wall following mode
    #  "turn left": Robot turns towards the left
    #  "search for wall": Robot tries to locate the wall        
    #  "follow wall": Robot moves parallel to the wall
    self.wall_following_state = "turn left"
         
    # Set turning speeds (to the left) in rad/s 
    # These values were determined by trial and error.
    self.turning_speed_wf_fast = 3.0  # Fast turn
    self.turning_speed_wf_slow = 0.05 # Slow turn
 
    # Wall following distance threshold.
    # We want to try to keep within this distance from the wall.
    self.dist_thresh_wf = 1.0 # in meters  
 
    # We don't want to get too close to the wall though.
    self.dist_too_close_to_wall = 0.19 # in meters

  def odom_callback(self, msg):
    """
    Receive the odometry information containing the position and orientation
    of the robot in the global reference frame. 
    The position is x, y, z.
    The orientation is a x,y,z,w quaternion. 
    """                    
    roll, pitch, yaw = self.euler_from_quaternion(
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z,
      msg.pose.pose.orientation.w)
  
    self.current_x = msg.pose.pose.position.x
    self.current_y = msg.pose.pose.position.y
    # On a horizontal plane, we do not need z
    self.current_z = msg.pose.pose.position.z
    
    # On a horizontal plane, we only need yaw
    self.current_roll = roll
    self.current_pitch = pitch
    self.current_yaw = yaw

    # Publish the estimated state (x position, y position, yaw angle)
    # obs_state_vector_x_y_yaw = [msg.pose.pose.position.x,msg.pose.pose.position.y,yaw]
    # self.publish_estimated_state(obs_state_vector_x_y_yaw)
  
  #  print(">>>>> odom_callback: ", msg.pose.pose.position.x,msg.pose.pose.position.y,yaw) 
  #
  #  # Command the robot to keep following the wall      
  #  #self.follow_wall()    

  def camera_callback(self, msg):
    try:
      im = pil_image.fromarray(self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'))

      img = ImageTk.PhotoImage(image=im)
      self.app.camera_label.configure(image=img)
      self.cv_image = img

    except CvBridgeError as e:
      print(e)

#  def state_estimate_callback(self, msg):
#    """
#    Extract the position and orientation data. 
#    This callback is called each time
#    a new message is received on the '/demo/state_est' topic
#    """
#    # Update the current estimated state in the global reference frame
#    curr_state = msg.data
#    self.current_x = curr_state[0]
#    self.current_y = curr_state[1]
#    self.current_yaw = curr_state[2]
#
#    print("###### state_estimate_callback: ", curr_state)
#
#    # Command the robot to keep following the wall      
#    self.follow_wall()    
        
  #def publish_estimated_state(self, state_vector_x_y_yaw):
  #  """
  #  Publish the estimated pose (position and orientation) of the 
  #  robot to the '/demo/state_est' topic. 
  #  :param: state_vector_x_y_yaw [x, y, yaw] 
  #  x is in meters, y is in meters, yaw is in radians
  #  """
  #  msg = Float64MultiArray()
  #  msg.data = state_vector_x_y_yaw
  #  self.publisher_state_est.publish(msg)

  def euler_from_quaternion(self, x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
 
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
 
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
 
    return roll_x, pitch_y, yaw_z # in radians
 
#  def velocity_callback(self, msg):
#    """
#    Listen to the velocity commands (linear forward velocity 
#    in the x direction in the robot's reference frame and 
#    angular velocity (yaw rate) around the robot's z-axis.
#    [v,yaw_rate]
#    [meters/second, radians/second]
#    """
#    # Forward velocity in the robot's reference frame
#    v = msg.linear.x
# 
#    # Angular velocity around the robot's z axis
#    yaw_rate = msg.angular.z
#
#    print(">>>>> velocity_callback: ", v,yaw_rate)

  def scan_callback(self, msg):
    """
    This method gets called every time a LaserScan message is 
    received on the '/demo/laser/out' topic 
    """
    # Read the laser scan data that indicates distances
    # to obstacles (e.g. wall) in meters and extract
    # 5 distinct laser readings to work with.
    # Each reading is separated by 45 degrees.
    # Assumes 181 laser readings, separated by 1 degree. 
    # (e.g. -90 degrees to 90 degrees....0 to 180 degrees)
 
    #number_of_laser_beams = str(len(msg.ranges))       
    self.left_dist = msg.ranges[90]
    self.leftfront_dist = msg.ranges[135]
    self.front_dist = msg.ranges[180]
    self.rightfront_dist = msg.ranges[225]
    self.right_dist = msg.ranges[270]

    self.arrLidarRanges = [ msg.ranges[225], msg.ranges[180], msg.ranges[135], 
      msg.ranges[270], 0.0, msg.ranges[90], 
      msg.ranges[315], msg.ranges[0], msg.ranges[45]]
#    print("B:%.2f, RB:%.2f, L:%.2f, LF:%.2f, F:%.2f, RF:%.2f, R:%.2f, LB:%.2f, " % (
#      msg.ranges[0], msg.ranges[45], msg.ranges[90], msg.ranges[135], 
#      msg.ranges[180], msg.ranges[225], msg.ranges[270], msg.ranges[315]), 
#      flush=True)

#    print("###### left:", self.left_dist, "leftfront:", self.leftfront_dist, 
#      "front:", self.front_dist, 
#      "rightfront:", self.rightfront_dist, "right:", self.right_dist)
        
    self.follow_wall(-1)   
             
  def follow_wall(self, nCommand):
    if(self.bManualControl and nCommand == -1):
      return
    # Create a geometry_msgs/Twist message
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0        
 
    # Logic for following the wall
    # >d means no wall detected by that laser beam
    # <d means an wall was detected by that laser beam
    d = self.dist_thresh_wf

    if(nCommand == 1 or (nCommand == -1 and self.leftfront_dist > d and 
      self.front_dist > d and 
      self.rightfront_dist > d)):
      self.wall_following_state = "go forward"
      msg.linear.x = self.forward_speed
      msg.angular.z = 0.
    elif(nCommand == 2):
      msg.linear.x = 0.
      msg.angular.z = -self.turning_speed_wf_fast
    else:
      msg.linear.x = 0.
      msg.angular.z = self.turning_speed_wf_fast

    # Send velocity command to the robot
    self.publisher_.publish(msg)    

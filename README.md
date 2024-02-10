# ROS-Robotics
This contains all the files need to successful complie a basic GUI inside Ubuntu Virtual Machine, and 
with ROS-2, GAZEBO installed. The tutorials for installing these including setting up the virtual machine please refer McGill Robotics's offical github at: https://github.com/mcgill-robotics/drone_2024/wiki.

Also, in order to successful complie, do the following in the terminal inside the virtual machine:

# In Terminal 1, bring up GUI Dashboard:
$ colcon build --packages-select teleop_gui
$ source install/setup.bash
$ ros2 launch teleop_gui teleop_gui.launch.py

# In Terminal 2, bring up Gazebo:
$ colcon build --packages-select diff_drive_controller_bot
$ source install/setup.bash
$ ros2 launch diff_drive_controller_bot launch_sim.launch world:=src/worlds/maze.sdf

If you face any error, please refer to the following code:

to run gui:
cd (into directory where it is located)

colcon build

source install/setup.bash

python3 gui_topic.py

HERE IS THE CODE FOR THE GUI(UNFINISHED):

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from threading import Thread

class ROSNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.publisher = self.create_publisher(String, 'chatter', 10)

    def publish_message(self, message):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)
        self.get_logger().info(f"I sent: {msg.data}")

def start_ros2_node(node):
    rclpy.spin(node)

def stop_ros2_node(node):
    node.destroy_node()
    rclpy.shutdown()

class ROS2GUI(tk.Tk):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.title("ROS2 GUI")

        self.message_entry = tk.Entry(self)
        self.message_entry.pack(pady=10)

        publish_button = tk.Button(self, text="Publish Message", command=self.publish_ros_message)
        publish_button.pack(pady=10)

        #####
        #publish_button = tk.Button(self, text="Publish Message", command=self.publish_ros_message)
        #publish_button.pack(pady=10)

        #####    
        start_button = tk.Button(self, text="Start ROS2 Node", command=self.start_ros2)
        start_button.pack(pady=10)

        stop_button = tk.Button(self, text="Stop ROS2 Node", command=self.stop_ros2)
        stop_button.pack(pady=10)

    def publish_ros_message(self):
        message = self.message_entry.get()
        if message:
            self.node.publish_message(message)

    def start_ros2(self):
        self.ros_thread = Thread(target=start_ros2_node, args=(self.node,))
        self.ros_thread.start()

    def stop_ros2(self):
        stop_ros2_node(self.node)
        self.ros_thread.join()

if __name__ == '__main__':
    rclpy.init()
    ros_node = ROSNode('minimal_publisher')
    gui = ROS2GUI(ros_node)
    gui.mainloop()



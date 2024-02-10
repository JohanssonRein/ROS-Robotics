# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modified by robotics.snowcron.com

# This is all-in-one launch script intended for use by nav2 developers.
# Attn: in ROS2 examples, this file is called "tb3_simulation_launch.py"

#import os
#from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node

import xacro

import sys
sys.path.append("src/navigation_bot_11/launch") 
from globals import *

def generate_launch_description():
    # Now in globals.py
    # package_name = 'navigation_bot_11'
    # bringup_dir = get_package_share_directory(package_name)
    
    # Now in globals.py
    # launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    keepout_mask_yaml_file = LaunchConfiguration('keepout_mask')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world', default=world_path)

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    # Now in globals.py
    #remappings = [('/tf', 'tf'),
    #              ('/tf_static', 'tf_static')]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    # maps_path declared in globals.py
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=maps_path,
        description='Full path to map file to load')

    # keepout_mask_path declared in globals.py
    declare_keepout_mask_yaml_cmd = DeclareLaunchArgument(
        'keepout_mask',
        default_value=keepout_mask_path,
        description='Full path to keepout_map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    # nav2_params_path declared in globals.py
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # nav2_bt_navigator_path declared in globals.py
    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=nav2_bt_navigator_path,
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the nav2 stack')

    # rviz_path defined in globals.py
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=rviz_path,
        description='Full path to the RVIZ config file to use')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')

    # world_path defined in globals.py
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        # TODO(orduno) Switch back once ROS argument passing has been fixed upstream
        #              https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/91
        # default_value=os.path.join(get_package_share_directory('turtlebot3_gazebo'),
        #                            'worlds/turtlebot3_worlds/waffle.model'),
        default_value=world_path,
        description='Full path to world model file to load')

    # Replaced by a single call below
    # start_gazebo_server_cmd = ExecuteProcess(
    #     condition=IfCondition(use_simulator),
    #     cmd=['gzserver', '-s', 'libgazebo_ros_init.so', world],
    #     cwd=[launch_dir], output='screen')
    # 
    # start_gazebo_client_cmd = ExecuteProcess(
    #     condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
    #     cmd=['gzclient'],
    #     cwd=[launch_dir], output='screen')

    # Alternative:
    # Include the Gazebo launch file, provided by the gazebo_ros package
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    #         launch_arguments={ 'gui':'True', 'server':'True' }.items() 
    #     )
    
    print(">>>", os.path.join(get_package_share_directory('navigation_bot_11')))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'gui': 'True',
            'server': 'True',
            #'models': os.path.join(get_package_share_directory('navigation_bot_11'), 'models')
        }.items()
    )    

    # Replaced by a single call below
    #start_robot_state_publisher_cmd = Node(
    #    condition=IfCondition(use_robot_state_pub),
    #    package='robot_state_publisher',
    #    executable='robot_state_publisher',
    #    name='robot_state_publisher',
    #    namespace=namespace,
    #    output='screen',
    #    parameters=[{'use_sim_time': use_sim_time}],
    #    remappings=remappings,
    #    arguments=[urdf])

    # Alternative:
    robot_description_config = xacro.process_file(urdf) #xacro_file)
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        #name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        remappings=remappings,
        parameters=[params]
    )    

    # Run the spawner node from the gazebo_ros package. 
    # The entity name doesn't really matter if you only have a single robot.
    spawn_entity_cmd = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
            '-entity', 'navigation_bot_11'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    #robot_localization_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[os.path.join(pkg_path, 'config/ekf.yaml')]#, {'use_sim_time': use_sim_time }]
    #)    

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'keepout' : keepout_mask_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'default_bt_xml_filename': default_bt_xml_filename,
                          'autostart': autostart}.items())
    bringup_timer_action = launch.actions.TimerAction( period=5.0, actions=[ bringup_cmd ])
    
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False',
                          'rviz_config': rviz_config_file}.items())

    rviz_timer_action = launch.actions.TimerAction( period=3.0, actions=[ rviz_cmd ])

    params = { 'use_sim_time': use_sim_time}
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params],
        #condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_keepout_mask_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_autostart_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add any conditioned actions
    #ld.add_action(start_gazebo_server_cmd)
    #ld.add_action(start_gazebo_client_cmd)
    ld.add_action(gazebo)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(spawn_entity_cmd)
        
    #ld.add_action(rviz_cmd)
    ld.add_action(rviz_timer_action)
    
    #ld.add_action(bringup_cmd)
    ld.add_action(bringup_timer_action)

    return ld
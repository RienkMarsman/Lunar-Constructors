import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # This has to match robot name in Xacro file
    robotXacroName = 'differential_drive_robot'
    print("hello")
    print("hello")
    # This is the name of our package at the same time this is the name of the folder used to define paths
    namePackage = 'mobile_robot'
    
    # Relative path to the Xacro file defining the model
    modelFileRelativePath = 'model/robot.xacro'
    # Relative path to the Gazebo world file
    worldFileRelativePath = 'model/empty_world.world'
    
    # Absolute path to the model
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    #pathModelFile = '/home/mikolaj/ws_mobile/src/mobile_robot/model'
    # Absolute path to the world model
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)
    print("Model File Path:", pathModelFile)
    print("ding dong")
    print("World File Path:", pathWorldFile)
    # Get the robot description from the Xacro model file
    robotDescription = xacro.process_file(pathModelFile).toxml()
    print("debug 1")
    # This is the launch file from the gazebo_ros package
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    )
    print("debug 2")
    # Launch description
    gazeboLaunch = IncludeLaunchDescription(
    gazebo_rosPackageLaunch, 
    launch_arguments={'world': pathWorldFile}.items())

    
    
    
    
    
    
    
    # Here we create a gazebo_ros Node
    spawnModelNode = Node(
            package='gazebo_ros', 
            executable='spawn_entity.py', 
            arguments=['-topic', 'robot_description', '-entity', robotXacroName],
            output='screen'
    )
    # Robot state publisher Node
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
    )
        
    # Create an empty launch description object
    launchDescriptionObject = LaunchDescription()
    
    # Add gazebo launch
    launchDescriptionObject.add_action(gazeboLaunch)
    
    # Add the two models
    launchDescriptionObject.add_action(spawnModelNode)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    
    return launchDescriptionObject


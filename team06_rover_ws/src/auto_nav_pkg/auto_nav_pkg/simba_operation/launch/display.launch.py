from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument 
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import os


def generate_launch_description():
    ld = LaunchDescription()


    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'leo_gz_worlds'
    gazebo_name = get_package_share_directory("simba_operation")


    # Set ignition resource path (so it can find your world files)
    ign_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',
    value=[os.path.join(get_package_share_directory(pkg_name),'worlds')])

    
    #######################################
    robot_ns = DeclareLaunchArgument(
        "robot_ns",
        default_value="",
        description="Robot namespace",
    )
    #######################################

    # Include extra models in the world
    sdf_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'leo_empty.sdf')

    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        gz_world_path =  os.environ['IGN_GAZEBO_RESOURCE_PATH'] + os.pathsep + os.path.join(get_package_share_directory(pkg_name), "worlds")
    else:
        gz_world_path =  os.path.join(get_package_share_directory(pkg_name), "worlds")

    ign_resource_path_update = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',value=[gz_world_path])

    # Include the Gazebo launch file
    launch_gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
    launch_arguments={
    'gz_args' : '-r empty.sdf'
    }.items(),
    )

    # Add features
    gz_spawn_objects = Node(package='ros_gz_sim', executable='create',
    arguments=['-file', sdf_path,
    '-x', '2.0',
    '-y', '0.5',
    '-z', '0.0'],
    output='screen'
    )

    #######################################
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_name, "launch", "spawn.launch.py")
        ),
        launch_arguments={"robot_ns": LaunchConfiguration("robot_ns")}.items(),
    )
    
    # Bridge ROS topics and Gazebo messages for establishing communication
    topic_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        parameters=[
            {
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )
    #######################################
    '''
    # joint state publisher (GUI) node
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )
    '''
    # Rviz node
    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('gz_example_robot_description'), 'rviz', 'nav2.rviz')]
    )
    
    
    

    
    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(ign_resource_path)
    ld.add_action(ign_resource_path_update)
    
    ld.add_action(robot_ns)
    ld.add_action(launch_gazebo)
    ld.add_action(gz_spawn_objects)
    
    ld.add_action(spawn_robot)
    ld.add_action(topic_bridge)
    
    #ld.add_action(joint_state_publisher)
    ld.add_action(node_rviz)
    
    #ld.add_action(teleop_twist_keyboard_node)
    #ld.add_action(twist_mux_node)

    return ld

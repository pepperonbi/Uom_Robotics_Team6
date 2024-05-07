from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import SetParameter, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription()

    # Parameters, Nodes and Launch files go here

    # Declare package directory
    pkg_nav_demos = get_package_share_directory('simba_simulation')

    # Necessary fixes
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    lifecycle_nodes = [
        'amcl',
        'controller_server',
        'planner_server',
        'behaviour_server',
        'bt_navigator',

    ]

    # LOAD PARAMETERS FROM YAML FILES
    config_bt_nav     = PathJoinSubstitution([pkg_nav_demos, 'config', 'bt_nav.yaml'])
    config_planner    = PathJoinSubstitution([pkg_nav_demos, 'config', 'planner.yaml'])
    config_controller = PathJoinSubstitution([pkg_nav_demos, 'config', 'controller.yaml'])

    # Include Gazebo Simulation
    launch_gazebo = IncludeLaunchDescription(
     #PythonLaunchDescriptionSource([get_package_share_directory('simba_simulation'), '/launch', '/display.launch.py']),
     PythonLaunchDescriptionSource([get_package_share_directory('gz_example_robot_description'), '/launch', '/sim_robot.launch.py']),
    launch_arguments={}.items(),
    )
    
    # EKF Localization Node
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[PathJoinSubstitution([pkg_nav_demos, 'config', 'ekf.yaml'])]
    )

    # Include SLAM Toolbox standard launch file
    launch_slamtoolbox = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('slam_toolbox'), '/launch', '/online_async_launch.py']),
    launch_arguments={}.items(),
    )

    # Behaviour Tree Navigator
    node_bt_nav = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[config_bt_nav],
        #remappings=[('/cmd_vel', '/nav_cmd_vel')],
    )

    # Behaviour Tree Server
    node_behaviour = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behaviour_server',
        output='screen',
        parameters=[config_bt_nav],
        #remappings=[('/cmd_vel', '/nav_cmd_vel')],
    )
    
        # Planner Server Node
    node_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[config_planner],
        #remappings=[('/cmd_vel', '/nav_cmd_vel')],
    )
    
    # Controller Server Node
    node_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[config_controller],
        #remappings=[('/cmd_vel', '/nav_cmd_vel')],
    )
    
    # AMCL Server Node
    node_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[config_bt_nav],
        #remappings=[('/cmd_vel', '/nav_cmd_vel')],
    )
    
    
    # Lifecycle Node Manager to automatically start lifecycles nodes (from list)
    node_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
    )
    
    
    
    ###############################################################
    # EXPLORATION
    node_explore_lite = IncludeLaunchDescription(
     PythonLaunchDescriptionSource([get_package_share_directory('explore_lite'), '/launch', '/explore.launch.py']),
    launch_arguments={}.items(),
    )
    ###############################################################
    
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            {
                'output_topic': '/cmd_vel',
                'topics': {
                    'keyboard': {
                        'topic': '/teleop_cmd_vel',
                        'timeout': 0.5,
                        'priority': 15
                    },
                    'locks': {
                        'auto_nav': {
                            'topic': '/nav_cmd_vel',
                            'timeout': 0.0,
                            'priority': 10
                        }
                    }
                }
            }
        ],
        remappings={('/cmd_vel_out', '/cmd_vel')},
    )
    
    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(launch_gazebo)
    #ld.add_action(robot_localization_node)
    ld.add_action(launch_slamtoolbox)
    ld.add_action(node_bt_nav)
    ld.add_action(node_behaviour)
    ld.add_action(node_planner)
    ld.add_action(node_controller)
    ld.add_action(node_amcl)
    ld.add_action(node_lifecycle_manager)
    
    #ld.add_action(twist_mux_node)
    ld.add_action(node_explore_lite)
    return ld


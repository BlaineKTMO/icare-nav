from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # Launch DWA navigation node
        Node(
            package='simple_nav2',
            executable='dwa_nav_node',
            name='dwa_nav_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'max_vel_x': 0.5,
                'max_vel_theta': 1.0,
                'acc_x': 0.2,
                'acc_theta': 0.5,
                'goal_tolerance': 0.1,
                'obstacle_min_dist': 0.3
            }]
        )
    ]) 
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('simple_nav2')
    
    # Declare the use_sim_time parameter
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Create the simple navigation node
    simple_nav_node = Node(
        package='simple_nav2',
        executable='simple_nav_node',
        name='simple_nav_node',
        output='screen'
    )

    # Create the costmap node
    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'global_frame': 'base_link',
            'robot_base_frame': 'base_link',
            'update_frequency': 1.0,
            'publish_frequency': 0.5,
            'rolling_window': True,
            'width': 3,
            'height': 3,
            'resolution': 0.05,
            'robot_radius': 0.3,
            'plugins': ['voxel_layer', 'inflation_layer'],
            'voxel_layer': {
                'plugin': 'nav2_costmap_2d::VoxelLayer',
                'enabled': True,
                'publish_voxel_map': True,
                'origin_z': 0.0,
                'z_resolution': 0.05,
                'z_voxels': 16,
                'max_obstacle_height': 2.0,
                'mark_threshold': 1,
                'observation_sources': 'scan',
                'scan': {
                    'sensor_frame': 'base_link',
                    'data_type': 'LaserScan',
                    'topic': '/scan',
                    'marking': True,
                    'clearing': True
                }
            },
            'inflation_layer': {
                'plugin': 'nav2_costmap_2d::InflationLayer',
                'cost_scaling_factor': 3.0,
                'inflation_radius': 0.55
            }
        }]
    )

    # Create RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-d', os.path.join(pkg_dir, 'config', 'nav_view.rviz')]
    )

    return LaunchDescription([
        use_sim_time_arg,
        simple_nav_node,
        costmap_node,
        rviz_node
    ])
 
import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():    
    config_yaml_fusion = os.path.join(
        get_package_share_directory('resple'),
        'config',
        'config_ntu_day_01_livox.yaml')    
    config_rviz = os.path.join(
        get_package_share_directory('resple'),
        'config',
        'config.rviz')     
    return launch.LaunchDescription([          	
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', config_rviz, '--ros-args', '--log-level', 'WARN']),   
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='log',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'my_frame', '--ros-args', '--log-level', 'WARN']),                       
        launch_ros.actions.Node(
            package='resple',
            executable='RESPLE',
            name='RESPLE',
            emulate_tty=True,
            output='log',
            parameters=[config_yaml_fusion]),
        launch_ros.actions.Node(
            package='resple',
            executable='Mapping',
            name='Mapping',
            emulate_tty=True,
            output='log',
            parameters=[config_yaml_fusion])                                    
  ])


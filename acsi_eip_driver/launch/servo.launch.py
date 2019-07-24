import os
import launch
import launch_ros.actions

def generate_launch_description():

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            node_name='servo_node',
            package='acsi_eip_driver',
            node_executable='servo_node',
            output='log',
            parameters=[{'debug': False,
                         'host': '192.168.100.10',
                         'publish_joint_state': False,
                         'joint_name': 'drive1',
                         'joint_states_topic': 'joint_states',
                         'default_velocity': 24.169,
                         'default_accel': 966.77,
                         'default_decel': 966.77,
                         'default_force': 100.0,
                         }],
            ),
    ])
    


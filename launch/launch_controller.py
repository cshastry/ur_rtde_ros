import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='ur_rtde_ros',
            namespace='ur',
            executable='ur_control',
            parameters=[
                {"hostname": "192.168.122.47"}
            ]
        )
    ])
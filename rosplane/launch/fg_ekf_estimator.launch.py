import os
import sys
import launch.actions
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Create the package directory
    rosplane_dir = get_package_share_directory('rosplane')

    # Determine the appropriate control scheme.
    aircraft = "anaconda" # Default aircraft

    for arg in sys.argv:
        if arg.startswith("aircraft:="):
            aircraft = arg.split(":=")[1]

    autopilot_params = os.path.join(
        rosplane_dir,
        'params',
        aircraft + '_autopilot_params.yaml'
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Whether or not to use the /clock topic in simulation to run timers."
        ),
        Node(
            package='rosplane',
            executable='estimator_ekf',
            name='estimator_ekf',
            output='screen',
            parameters=[
                autopilot_params,
                {'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')},
            ]
        ),
        Node(
            package='rosplane',
            executable='estimator_fg',
            name='estimator_fg',
            output='screen',
            parameters=[
                autopilot_params,
                {'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')},
            ]
        ),
        Node(
            package='dummy_sensors',
            executable='dummy_sensors',
            name='dummy_sensors'
        )
    ])


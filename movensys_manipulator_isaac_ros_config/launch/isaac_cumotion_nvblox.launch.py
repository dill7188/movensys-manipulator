import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    camera_tf_arg_defaults = {
        "camera_0_x": "-0.397",
        "camera_0_y": "-0.239",
        "camera_0_z": "0.959",
        "camera_0_roll": "-0.226",
        "camera_0_pitch": "1.087",
        "camera_0_yaw": "0.456",
        "camera_1_x": "0.347",
        "camera_1_y": "-0.365",
        "camera_1_z": "1.019",
        "camera_1_roll": "-0.056",
        "camera_1_pitch": "0.677",
        "camera_1_yaw": "2.423",
    }

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock (/clock)"
    )

    declare_rsp = DeclareLaunchArgument(
        "rsp",
        default_value="true",
        description="Publish /robot_description via isaac_cumotion.launch.py. Set false "
                    "when a backend launch (Gazebo sim or wmx_r2_control) already publishes it."
    )
    declare_publish_camera_tf = DeclareLaunchArgument(
        "publish_camera_tf",
        default_value="true",
        description="Publish static camera TF. Set false while tuning camera TF with GUI."
    )
    camera_tf_args = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in camera_tf_arg_defaults.items()
    ]

    pkg_movensys_isaac_ros_config = get_package_share_directory(
        'movensys_manipulator_isaac_ros_config')
    pkg_movensys_manipulator_perception = get_package_share_directory(
        'movensys_manipulator_perception')

    nvblox_rviz = os.path.join(pkg_movensys_isaac_ros_config, 'rviz', 'nvblox.rviz')

    camera_nvblox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_movensys_manipulator_perception, 'launch', 'camera_nvblox.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'publish_camera_tf': LaunchConfiguration('publish_camera_tf'),
            **{
                name: LaunchConfiguration(name)
                for name in camera_tf_arg_defaults
            },
        }.items(),
    )

    cumotion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_movensys_isaac_ros_config, 'launch', 'isaac_cumotion.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'read_esdf_world': 'true',
            'rviz_config': nvblox_rviz,
            'rsp': LaunchConfiguration('rsp'),
        }.items()
    )

    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_movensys_isaac_ros_config, 'launch', 'isaac_nvblox.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_rsp,
        declare_publish_camera_tf,
        *camera_tf_args,
        camera_nvblox,
        cumotion_launch,
        nvblox_launch,
    ])

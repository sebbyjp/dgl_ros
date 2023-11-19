# ROS2 Launch file for inference node.
# <node name="grasp_cloud_detection" pkg="moveit_task_constructor_gpd" type="grasp_cloud_detection" output="screen" launch-prefix="$(arg launch_prefix)">
#     <!-- <node ns="locobot" name="grasp_cloud_detection" pkg="moveit_task_constructor_gpd" type="grasp_cloud_detection" output="screen"> -->
#     <param if="$(arg load_cloud)" name="path_to_pcd_file" value="$(arg path_to_pcd_file)" />
#     <param unless="$(arg load_cloud)" name="point_cloud_topic" value="$(arg point_cloud_topic)" />
#     <rosparam command="load" file="$(find deep_grasp_task)/config/calib/camera.yaml" />
#     <rosparam param="view_point">[0, 0, 0]</rosparam>
#     <param name="load_cloud" value="$(arg load_cloud)" />
#     <param name="action_name" value="$(arg action_name)" />
#     <param name="frame_id" value="$(arg frame_id)" />
#     <param name="path_to_gpd_config" value="$(arg path_to_gpd_config)" />
#   </node>

from os import path
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
    FindExecutable,
    Command,
    LaunchConfiguration,
)
from pathlib import Path
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch.action import Action
from ament_index_python.packages import get_package_share_directory
import yaml


def generate_launch_description() -> LaunchDescription:
    deep_grasp_task_dir = get_package_share_directory('deep_grasp_task')
    with open(path.join(deep_grasp_task_dir, 'config/block_config.yaml')) as f:
        task_config = yaml.load(f, Loader=yaml.FullLoader)

    return LaunchDescription([
        # TODO(speralta): Add launch argument for yaml config of input topics.
        DeclareLaunchArgument('input_topic',
                              default_value='/rgbd_camera/points',
                              description='The sensor topic to subscribe to. '),
        DeclareLaunchArgument(
            'input_type',
            default_value='sensor_msgs/msg/Pointcloud2',
            description='whether to use cylinder segmentation.',
        ),
        Node(
            name="detect_grasps_node",
            package='ros_dgl_core',
            executable='grasp_detection_server',
            output='screen',
            parameters=[{
                "input_topic": LaunchConfiguration('input_topic')
            }],
        )
    ])

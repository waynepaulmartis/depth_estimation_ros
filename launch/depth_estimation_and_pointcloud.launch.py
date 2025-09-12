from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros
from launch.conditions import IfCondition


def generate_launch_description():

    declare_topic_name = DeclareLaunchArgument(
        'topic_name',
        default_value='/camera/color/image_raw',
        description='Name of the topic to subscribe to'
    )
    
    zenoh_arg = DeclareLaunchArgument(
        'zenoh',
        default_value='true',
        description='Flag to enable Zenoh router'
    )
    
    zenoh_router = Node(
        package='rmw_zenoh_cpp',
        executable='rmw_zenohd',
        name='zenoh_router',
        output='screen',
        condition=IfCondition(LaunchConfiguration('zenoh'))
    )

    depth_map_node = Node(
        package='depth_estimation_ros',
        namespace='',
        executable='depth_map_to_pointcloud_colorised',
        name='depth_map_to_pointcloud_colorised',
        parameters=[{
            'topic_name': LaunchConfiguration('topic_name'),
        }]
    )

    depth_estimation_node = Node(
        package='depth_estimation_ros',
        namespace='',
        executable='depth_estimation_onnx_multi_node',
        name='depth_estimation_onnx_multi_node',
        output='screen'
    )

    return LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=False),
        declare_topic_name,
        zenoh_arg,
        zenoh_router,
        depth_estimation_node,
        depth_map_node,
    ])

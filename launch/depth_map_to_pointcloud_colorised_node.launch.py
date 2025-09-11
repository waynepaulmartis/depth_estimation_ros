from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    declare_topic_name = DeclareLaunchArgument(
        'topic_name',
        default_value='/camera/color/image_raw',
        description='Name of the topic to subscribe to')
    
    zenoh_arg = DeclareLaunchArgument(name='zenoh',default_value='true',choices=['true', 'false'],
        description='Flag to enable Zenoh router'    )
    
    zenoh_router = Node(
        package='rmw_zenoh_cpp',
        executable='rmw_zenohd',
        name='zenoh_router',
        output='screen',
        condition=IfCondition(LaunchConfiguration('zenoh'))
    )

 
    

    return LaunchDescription([
        # Use this to activate sim time
        launch_ros.actions.SetParameter(name='use_sim_time', value=False),

        declare_topic_name,
        zenoh_arg,
        zenoh_router,
        

        # Launch Camera 
        Node(
            package='depth_estimation_ros',
            namespace='',
            executable='depth_map_to_pointcloud_colorised',
            name='depth_map_to_pointcloud_colorised',
            parameters=[{
                'topic_name': LaunchConfiguration('topic_name'),

            }]
        ),


    ])

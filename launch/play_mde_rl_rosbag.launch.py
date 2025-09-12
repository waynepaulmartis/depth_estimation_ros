from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path
from launch.actions import ExecuteProcess


def generate_launch_description():

    declare_topic_name = DeclareLaunchArgument(
        'topic_name',
        default_value='/camera/color/image_raw',
        description='Name of the topic to subscribe to')
    
    zenoh_arg = DeclareLaunchArgument(name='zenoh',default_value='true',choices=['true', 'false'],
        description='Flag to enable Zenoh router'    )
    
    play_bag_arg = DeclareLaunchArgument(
        'play_bag', default_value='true', choices=['true','false'],
        description='Play rosbag'
    )

    rosbag_path_arg = DeclareLaunchArgument(
        'rosbag_path',
        default_value='/home/admin-jfinke/hunter_se_camera_calibration/processing/test_bag_2',
        description='Path to rosbag directory (contains metadata.yaml)'
    )

    zenoh_router = Node(
        package='rmw_zenoh_cpp',
        executable='rmw_zenohd',
        name='zenoh_router',
        output='screen',
        condition=IfCondition(LaunchConfiguration('zenoh'))
    )

    robomaster_pkg_share = get_package_share_path('ros_robomaster_description')
    robomaster_launch_file = robomaster_pkg_share / 'launch' / 'ros_robomaster_description.launch.py'
    

    ## Play the rosbag with the camera data at location rosbag_path and loop it
    play_rosbag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('rosbag_path'), '--loop'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('play_bag'))
    )





    include_robomaster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(robomaster_launch_file)),
        launch_arguments={
            'namespace': 'rmwayne',
            'use_sim_time': 'false',
            'prefix': 'rmwayne/',
            'include_mde_luxonis_cameras': 'true'
        }.items()
    )




    return LaunchDescription([
        # Use this to activate sim time
        launch_ros.actions.SetParameter(name='use_sim_time', value=False),
        rosbag_path_arg,
        play_bag_arg,
        declare_topic_name,
        zenoh_arg,
        zenoh_router,
        include_robomaster,
        play_rosbag,  # <-- Add this line!

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

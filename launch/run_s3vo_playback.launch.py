"""Run avoa3d's s3vo pipeline against a played-back bag, for offline analysis
of how the avoidance samples behaved at recording time.

Two problems with pointing avoa3dnode/rviz_marker straight at the bag:

1. Their cmd_vel output should not go anywhere real during playback -- we
   only want to look at desired_vel vs. cmd_vel vs. element_tracking, not
   drive anything. 'topics.cmd_vel' is remapped to a separate '*_after'
   debug topic instead of the vehicle's real /usv/lily/cmd_vel.

2. avoa3dnode/rviz_marker subscribe to the odometry topic with rclcpp's
   default QoS (RELIABLE), but the bag's /usv/lily/mavros/local_position/odom
   was recorded BEST_EFFORT (mavros' default). That QoS mismatch means the
   subscription silently never receives anything when the bag is played.
   odom_reliable_relay bridges it onto a reliable topic that avoa3d is
   pointed at instead. See odom_reliable_relay.py for details.

This never modifies the bag or the avoa3d package; it only adds a relay
node and points avoa3d's launch arguments at different topic names.

Usage:
  ros2 bag play <bag_dir> --clock
  ros2 launch s3vo_analysis run_s3vo_playback.launch.py
  rviz2   (Fixed Frame: world; also run patch_bag_tf.launch.py for TF)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use the bag/clock time (play the bag with --clock).')
    use_sim_time = LaunchConfiguration('use_sim_time')

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/usv/lily/mavros/local_position/odom')
    reliable_odom_topic_arg = DeclareLaunchArgument(
        'reliable_odom_topic', default_value='/usv/lily/mavros/local_position/odom_reliable')
    cmd_vel_after_topic_arg = DeclareLaunchArgument(
        'cmd_vel_after_topic', default_value='/usv/lily/cmd_vel_after',
        description="Where avoa3d's recomputed cmd_vel is published during "
                    "playback, instead of the real /usv/lily/cmd_vel.")

    # QoS bridge: bag's odom is BEST_EFFORT, avoa3d subscribes RELIABLE.
    odom_reliable_relay_node = Node(
        package='s3vo_analysis',
        executable='odom_reliable_relay',
        name='lily_odom_reliable_relay',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': LaunchConfiguration('odom_topic'),
            'output_topic': LaunchConfiguration('reliable_odom_topic'),
        }],
    )

    s3vo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('avoa3d'), 'launch', 's3vo.launch.py')
        ),
        launch_arguments={
            'topics_odometry': LaunchConfiguration('reliable_odom_topic'),
            'topics_cmd_vel': LaunchConfiguration('cmd_vel_after_topic'),
        }.items(),
    )

    return LaunchDescription([
        use_sim_time_arg,
        odom_topic_arg,
        reliable_odom_topic_arg,
        cmd_vel_after_topic_arg,
        odom_reliable_relay_node,
        s3vo_launch,
    ])

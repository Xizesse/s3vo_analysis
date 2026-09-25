"""Patch missing TF for the S3VO bags (deploy_p_*, trial_*).

The recorded bags have no /tf or /tf_static at all, but they do have every
vehicle's mavros local_position/odom and global_position/gp_origin. This
launch file runs a small set of nodes that reconstruct the TF tree from
those topics while a bag is being played -- it never modifies the bag.

Frames produced:
  world                       -- reference frame, anchored on lily's gp_origin
  world -> map                -- identity (lily's own EKF/local frame; this
                                  is the frame_id already used by lily's
                                  recorded odom/element_tracking messages)
  map -> base_link             -- lily's pose, from lily's odom
  world -> nautilus/map        -- static offset computed from both gp_origins
  nautilus/map -> nautilus/base_link -- nautilus's pose, from its own odom

This matches avoa3d/s3vo.launch.py's expectations (fixed_frame='world',
agent_frame='base_link' for lily) while also placing nautilus consistently
in the same world frame using its own recorded GPS origin.

It also publishes the (reverse-engineered, not recorded) goal position as
RViz markers: a small sphere at the goal and a line from lily's current
position to it, both in the 'world' frame.

Usage:
  ros2 bag play <bag_dir> --clock
  ros2 launch s3vo_analysis patch_bag_tf.launch.py
  rviz2   (Fixed Frame: world)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use the bag/clock time (play the bag with --clock).')
    use_sim_time = LaunchConfiguration('use_sim_time')

    world_frame_arg = DeclareLaunchArgument('world_frame', default_value='world')
    world_frame = LaunchConfiguration('world_frame')

    lily_odom_arg = DeclareLaunchArgument(
        'lily_odom_topic', default_value='/usv/lily/mavros/local_position/odom')
    nautilus_odom_arg = DeclareLaunchArgument(
        'nautilus_odom_topic', default_value='/usv/nautilus/mavros/local_position/odom')
    lily_gp_origin_topic = '/usv/lily/mavros/global_position/gp_origin'
    nautilus_gp_origin_topic = '/usv/nautilus/mavros/global_position/gp_origin'

    goal_latitude_arg = DeclareLaunchArgument('goal_latitude', default_value='41.6852940')
    goal_longitude_arg = DeclareLaunchArgument('goal_longitude', default_value='-8.8390284')

    # Anchors both vehicles' local frames into a common 'world' frame using
    # the GPS origins recorded in the bag. Lily is the reference (world ==
    # lily's local tangent plane), matching avoa3d's fixed_frame='world'.
    gps_origin_tf_node = Node(
        package='s3vo_analysis',
        executable='gps_origin_tf',
        name='gps_origin_tf',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'world_frame': world_frame,
            'vehicles': ['lily', 'nautilus'],
            'gp_origin_topics': [lily_gp_origin_topic, nautilus_gp_origin_topic],
            'local_frames': ['map', 'nautilus/map'],
            'reference_vehicle': 'lily',
        }],
    )

    # world -> map is identity by construction (lily is the reference), but
    # publish it explicitly so 'map' (the frame already used inside the bag's
    # own messages) is always in the tree even before gp_origin_tf resolves it.
    world_to_map_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_static_tf',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--frame-id', world_frame, '--child-frame-id', 'map'],
    )

    lily_odom_tf_node = Node(
        package='s3vo_analysis',
        executable='odom_tf_broadcaster',
        name='lily_odom_tf_broadcaster',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': LaunchConfiguration('lily_odom_topic'),
            'parent_frame': 'map',
            'child_frame': 'base_link',
        }],
    )

    nautilus_odom_tf_node = Node(
        package='s3vo_analysis',
        executable='odom_tf_broadcaster',
        name='nautilus_odom_tf_broadcaster',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': LaunchConfiguration('nautilus_odom_topic'),
            'parent_frame': 'nautilus/map',
            'child_frame': 'nautilus/base_link',
        }],
    )

    goal_marker_node = Node(
        package='s3vo_analysis',
        executable='goal_marker_publisher',
        name='goal_marker_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'goal_latitude': LaunchConfiguration('goal_latitude'),
            'goal_longitude': LaunchConfiguration('goal_longitude'),
            'reference_gp_origin_topic': lily_gp_origin_topic,
            'world_frame': world_frame,
            'agent_frame': 'base_link',
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_frame_arg,
        lily_odom_arg,
        nautilus_odom_arg,
        goal_latitude_arg,
        goal_longitude_arg,
        gps_origin_tf_node,
        world_to_map_static_tf,
        lily_odom_tf_node,
        nautilus_odom_tf_node,
        goal_marker_node,
    ])

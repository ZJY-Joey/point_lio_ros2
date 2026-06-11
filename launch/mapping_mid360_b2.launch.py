from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetParameter
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Flag to launch RViz.')
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time.')
    robot_arg = DeclareLaunchArgument(
        'robot', default_value='b2',
        description='Robot type passed to quadruped_tf.launch.xml.')

    laser_mapping_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('point_lio'),
                'config',
                'mid360_b2.yaml'
            ]),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_imu_as_input': True,
                'prop_at_freq_of_imu': True,
                'check_satu': True,
                'init_map_size': 10,
                'point_filter_num': 6,
                'space_down_sample': True,
                'filter_size_surf': 0.5,
                'filter_size_map': 0.5,
                'cube_side_length': 1000.0,
                'runtime_pos_log_enable': False,
                'ivox_nearby_type': 6,
                'location_mode': False,
            }
        ],
        prefix='taskset -c 0-3 chrt -f 99',
    )

    message_to_tf_node = Node(
        package='message_to_tf',
        executable='message_to_tf_node',
        name='message_to_tf',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'odometry_topic': '/aft_mapped_to_init',
            'frame_id': '/camera_init',
            'footprint_frame_id': '/aft_mapped_footprint',
            'stabilized_frame_id': '/aft_mapped_stabilized',
            'child_frame_id': '/aft_mapped'
        }],
        prefix='taskset -c 0-3 nice -n -10',
    )

    quadruped_tf = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'launch',
            'quadruped_tf.launch.xml'
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot': LaunchConfiguration('robot'),
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'rviz_cfg',
            'loam_livox.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice',
    )

    return LaunchDescription([
        rviz_arg,
        sim_time_arg,
        robot_arg,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        laser_mapping_node,
        message_to_tf_node,
        quadruped_tf,
        GroupAction(
            actions=[rviz_node],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ])

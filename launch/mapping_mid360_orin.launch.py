from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Flag to launch RViz.')
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time.')

    laser_mapping_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('point_lio'),
                'config',
                'mid360_orin.yaml'
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
        prefix='taskset -c 0-3 nice -n -10',
    )

    aft_mapped_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_aft_mapped_to_base',
        arguments=[
            '--x', '-0.2',
            '--y', '0.0',
            '--z', '0.0',
            '--yaw', '-1.5708',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'aft_mapped',
            '--child-frame-id', 'base',
        ],
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
        laser_mapping_node,
        aft_mapped_to_base_tf,
        GroupAction(
            actions=[rviz_node],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ])

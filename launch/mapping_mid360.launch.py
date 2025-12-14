from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare the RViz argument
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Flag to launch RViz.')
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time'
    )
    robot_arg = DeclareLaunchArgument(
        'robot', default_value=''
    )

    # Node parameters, including those from the YAML configuration file
    laser_mapping_params = [
        PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'config',
            PythonExpression([ "'mid360_' + '", LaunchConfiguration('robot'), "' + '.yaml'" ])
        ]),
        {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_imu_as_input': True,  # Change to True to use IMU as input of Point-LIO
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 10,
            'point_filter_num': 6,  # Options: 1, 3
            'space_down_sample': True,
            'filter_size_surf': 0.5,  # Options: 0.5, 0.3, 0.2, 0.15, 0.1
            'filter_size_map': 0.5,  # Options: 0.5, 0.3, 0.15, 0.1
            'cube_side_length': 1000.0,  # Option: 1000
            'runtime_pos_log_enable': False,  # Option: True
            'ivox_nearby_type': 6,
            'location_mode':False,
        }
    ]

    # Node definition for laserMapping with Point-LIO
    laser_mapping_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=laser_mapping_params,
        # prefix='gdb -ex run --args', chrt -f 50
        prefix='taskset -c 0-3 chrt -f 99',
    )

    message_to_tf_node = Node(
        package='message_to_tf',
        executable='message_to_tf_node',
        name='message_to_tf',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'odometry_topic': '/aft_mapped_to_init',  # Change to True to use IMU as input of Point-LIO
            'frame_id': '/camera_init',
            'footprint_frame_id': '/aft_mapped_footprint',
            'stabilized_frame_id': '/aft_mapped_stabilized',
            'child_frame_id': '/aft_mapped'
        }],
        prefix='taskset -c 0-3 nice -n -10',
    )

    # Conditional RViz node launch
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'rviz_cfg', 'loam_livox.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice'
    )


    static_tf_base2chassie = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base2chassie",
        arguments=["0.3410", "0.", "0.1779", "1.5708", "0.", "0.1745", "aliengo", "livox_frame"],
    )   
    static_tf_pose2base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_pose2base",
        arguments=["0.", "0.023", "-0.049", "0.", "0.", "0.", "aft_mapped", "aliengo"],
    )   

    static_tf_world2camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_pose2base",
        arguments=["0.0", "0.0", "0.0", "0.", "0.", "0.", "world", "camera_init"],
    )   


    pkg_share_dir = get_package_share_directory('point_lio')
    xml_launch_file_path = PathJoinSubstitution([
        pkg_share_dir,
        'launch',
        'quadruped_tf.launch.xml']
    )
    include_xml_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(xml_launch_file_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot': LaunchConfiguration('robot'),
        }.items()
    )

    # Assemble the launch description
    ld = LaunchDescription([
        rviz_arg,
        sim_time_arg,
        robot_arg,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        laser_mapping_node,
        message_to_tf_node,
        include_xml_launch,
        GroupAction(
            actions=[rviz_node],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ])

    return ld

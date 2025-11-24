from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare the RViz argument
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Flag to launch RViz.')
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
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
            'prop_at_freq_of_imu': False,
            'check_satu': True,
            'init_map_size': 10,
            'point_filter_num': 3   ,  # Options: 1, 3
            'space_down_sample': True,
            # TODO: changee parameters in need if experiments are not good
            'filter_size_surf': 0.5,  # Options: 0.5, 0.3, 0.2, 0.15, 0.1
            'filter_size_map': 0.5,  # Options: 0.5, 0.3, 0.15, 0.1
            'cube_side_length': 1000.0,  # Option: 1000
            'runtime_pos_log_enable': False,  # Option: True
            # localization parameters
            'location_mode':True,
            'initial_z': 0.0,
            'map_path': '/home/unitree/atecup_ws/maps/0405_csc1floor.pcd',
            'publish/scan_bodyframe_pub_en': True,
            'pcd_save/pcd_save_en': False,
        }
    ]

    # Node definition for laserMapping with Point-LIO
    laser_mapping_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=laser_mapping_params,
        prefix='taskset -c 2-7'
    )

    # Conditional RViz node launch
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'rviz_cfg', 'loam_livox_b2.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice -n 10'
    )


    tf_node = Node(
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
        }]
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

    static_tf_world2camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_world2camera",
        arguments=["0.0", "0.0", "0.0", "0.", "0.", "0.", "world", "camera_init"],
    )
    static_tf_base2chassie = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base2chassie",
        # arguments=["0.3410", "0.", "0.1779", "1.5708", "0.", "0.1745", "aliengo", "livox_frame"],  # unitree b2 lidar
        arguments=["0.", "0.", "0.04412", "1.5708", "0.", "0.1745", "aliengo", "livox_frame"],
    )   
    static_tf_pose2base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_pose2base",
        # arguments=["0.", "0.023", "-0.049", "0.", "0.", "0.", "aft_mapped", "aliengo"],  # unitree b2 imu
        arguments=["0.011", "0.02329", "0.", "-1.5708", "-0.1745", "0.", "aft_mapped", "aliengo"],
    )   

    # Assemble the launch description
    ld = LaunchDescription([
        rviz_arg,
        sim_time_arg,
        robot_arg,
        laser_mapping_node,
        tf_node,
        include_xml_launch,
        # static_tf_base2chassie,
        # static_tf_pose2base,
        # static_tf_world2camera,
        GroupAction(
            actions=[rviz_node],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ])

    return ld

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart



def generate_launch_description():
    package_name='vacuum_bot'
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    coverage_config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'opennav_params.yaml'
    )
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
        )    
    
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )
    
    start_x_arg = DeclareLaunchArgument(
        'start_x', default_value='-1.62', description='Posizione iniziale X'
    )
    start_y_arg = DeclareLaunchArgument(
        'start_y', default_value='-5.11', description='Posizione iniziale Y'
    )
    start_theta_arg = DeclareLaunchArgument(
        'start_theta', default_value='1.57', description='Orientamento iniziale Theta'
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    
    # Start robot state publisher
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )
    
    # Start joystick
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','joystick.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Start /diff_cont/cmd_vel multiplexer
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel')]
        )

    # Start the simulation
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )
    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_bot',
            '-x', LaunchConfiguration('start_x'),
            '-y', LaunchConfiguration('start_y'),
            '-z', '0.1',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', LaunchConfiguration('start_theta')
        ],
        output='screen')

    # Start ros2 controllers
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Start step controller
    step_controller_node = Node(
        package=package_name,
        executable='sim_step_controller_node',
        parameters=[{'simulation': True}, {'use_sim_time': True}],
        output='screen'
    )
    delayed_step_controller_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_spawner,
            on_exit=[step_controller_node],
        )
    )   

    # Start Gazebo bridge
    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )
    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    # Start SLAM toolbox
    slam_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'mapper_params_online_async.yaml'
    )

    slam_toolbox_params = {
        'map_start_pose': [LaunchConfiguration('start_x'), 
                           LaunchConfiguration('start_y'), 
                           LaunchConfiguration('start_theta')]
    }

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'),
            'launch',
            'online_async_launch.py'
        )]),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'true',
            'params': slam_toolbox_params
        }.items()
    )

    # start the visualization

    # rviz_config_file = os.path.join(
    #     get_package_share_directory(package_name),
    #     'config',
    #     'map.rviz'
    # )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     arguments=["-d", rviz_config_file],
    #     output="screen"
    # )

    # delayed_rviz = TimerAction(
    #     period=5.0,   # aspetta 5 secondi
    #     actions=[rviz_node]
    # )

    rviz_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'opennav_coverage_demo.rviz',
    )
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={
            'namespace': '', 
            'rviz_config': rviz_config,
            'use_sim_time': 'true'
        }.items())

    delayed_rviz = TimerAction(
        period=5.0,   # aspetta 5 secondi
        actions=[rviz_cmd]
    )

    # start navigation

    nav2_params_file = os.path.join(
        get_package_share_directory('vacuum_bot'),
        'config',
        'nav2_params.yaml'
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','navigation_launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'params_file': nav2_params_file}.items()
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 
                         'launch',
                         'opennav_bringup_launch.py')),
        launch_arguments={'use_sim_time': 'true', 'params_file': coverage_config_file}.items())
    
    # If no localization system (es. AMCL, SLAM toolbox) is used, use this 
    # to publish a static transform map -> odom to make rviz and nav2 work. 
    fake_localization_cmd = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'])

    # Start the coverage task
    coverage_node = Node(
        package=package_name,
        executable='coverage_nav2_node',
        emulate_tty=True,
        output='screen',
        # arguments=['--ros-args', '--log-level', 'debug']
    )
    

    # Launch them all!
    return LaunchDescription([
        start_x_arg,
        start_y_arg,
        start_theta_arg,
        rsp,
        joystick,
        twist_mux,
        world_arg,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        ros_gz_bridge,
        ros_gz_image_bridge,
        step_controller_node,
        # nav2,
        slam_toolbox,
        # rviz_node,
        delayed_rviz,
        # rviz_cmd,
        bringup_cmd,
        # fake_localization_cmd,
        coverage_node
    ])

from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.actions import TimerAction

def generate_launch_description():
    # Путь к пакету inspector_gazebo
    pkg_inspector_gazebo = get_package_share_directory('spider_description')

    # Запуск мира Gazebo

    default_world = os.path.join(
        get_package_share_directory('spider_controllers'),
        'worlds',
        'empty.world'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', default_world], 'on_exit_shutdown': 'true'}.items()
    )

    # Загрузка модели робота
    robot_description = Command(['xacro ', str(os.path.join(pkg_inspector_gazebo, 'urdf', 'spider.urdf.xacro'))])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Спавн робота в Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'spider',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '1.0',
            
        ],
        output='screen'
    )

    # bridge_params = os.path.join(get_package_share_directory('inspector_gazebo'),'config','gz_bridge.yaml')
    # ros_gz_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     arguments=[
    #         '--ros-args',
    #         '-p',
    #         f'config_file:={bridge_params}',
    #     ]
    # )

    # ros_gz_image_bridge = Node(
    #     package="ros_gz_image",
    #     executable="image_bridge",
    #     arguments=["/camera/image_raw"]
    # )

    joint_broad_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    forward_command_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_command_controller'],
        output='screen'
    )

    activate_forward_command_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'set_controller_state', 'forward_command_controller', 'active'],
        output='screen'
    )
    spawn_robot_delayed = TimerAction(
    period=5.0,  # Ждём 5 секунд
    actions=[spawn_entity],
    )
    
    return LaunchDescription([
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=spawn_entity,
        #         on_exit=[forward_command_controller],
        #     )
        # ),

        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=forward_command_controller,
        #         on_exit=[joint_broad_spawner],
        #     )
        # ),

        gazebo,
        robot_state_publisher,
        spawn_robot_delayed,
       # ros_gz_bridge,
        #activate_forward_command_controller,  # Нужно для активации контроллера из-за 
                                              # ошибки ожидания переключения контроллеров более 5 секнунд
        # ros_gz_image_bridg
        
    ])

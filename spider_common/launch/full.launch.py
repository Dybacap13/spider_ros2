
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python import get_package_share_directory
import os

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path


def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():

# ****************************
#    Declared arguments      *
# ****************************
  declared_arguments = []

# Description package and files

  declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="spider_description",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
  declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="spider.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

  declared_arguments.append(
        DeclareLaunchArgument(
            "parametrs_package",
            default_value="spider_description",
            description="spider_parametrs.yaml.",
        )
    )

  declared_arguments.append(
        DeclareLaunchArgument(
            "parametrs_file",
            default_value="spider_parametrs.yaml",
            description="spider_parametrs.yaml",
        )
    )


# rviz

  declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
  

# controllers

  declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_package",
            default_value="spider_controllers",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
  declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ros_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )

# ****************************
#     General arguments      *
# ****************************

  description_package = LaunchConfiguration("description_package")
  description_file = LaunchConfiguration("description_file")
  controllers_package = LaunchConfiguration("controllers_package")
  controllers_file = LaunchConfiguration("controllers_file")
  launch_rviz = LaunchConfiguration("launch_rviz")

  parametrs_package = LaunchConfiguration("parametrs_package")
  parametrs_file = LaunchConfiguration("parametrs_file")



# ****************************
#  Robot description URDF    *
# ****************************

  robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            )
            ]
        )
  
  robot_description = {"robot_description": robot_description_content}  


# ****************************
#       Controllers          *
# ****************************

  robot_controllers = PathJoinSubstitution(
        [FindPackageShare(controllers_package), "config", controllers_file]
    )

# ****************************
#       Parametrs          *
# ****************************

  parametrs = PathJoinSubstitution(
        [FindPackageShare(parametrs_package), "config", parametrs_file]
    )
 
  container_parametrs = ComposableNodeContainer(
        name = 'spider_ik',
        namespace = '',
        package = 'rclcpp_components',
        executable = 'component_container_mt',
        composable_node_descriptions = [
            ComposableNode(
                package = 'spider_ik_server',
                plugin = 'spider_ik::IkServers',
                name = 'spider_ik', 
                namespace = 'spider_client_library',
                parameters = [parametrs]
            )
        ],
        output = 'screen'
    )

# ****************************
#       Nodes control        *
# ****************************

  control_node = Node(
              package="controller_manager",
              executable="ros2_control_node",
              parameters=[ {'robot_description': robot_description_content},
                          robot_controllers
                           
              ],
            emulate_tty=True,
          )

  rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=[
            '-d' + os.path.join(get_package_share_directory('spider_common'), 'cfg', 'rviz2.rviz')],

        parameters=[
            robot_description   
        ],
    )
  
  robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
        emulate_tty=True,
    )
  

# ****************************
#       Controllers          *
# ****************************

  joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[{'controller-manager-timeout': 120}, ],
        emulate_tty=True,
    )

  robot_controller_spawners = []
  robot_controllers_active =  ['joint_coxa_lf_controller', 'joint_femur_lf_controller', 'joint_tibia_lf_controller', 'joint_coxa_rf_controller', 'joint_femur_rf_controller', 'joint_tibia_rf_controller', 'joint_coxa_lm_controller', 'joint_femur_lm_controller', 'joint_tibia_lm_controller', 'joint_coxa_rm_controller', 'joint_femur_rm_controller', 'joint_tibia_rm_controller', 'joint_coxa_lr_controller', 'joint_femur_lr_controller', 'joint_tibia_lr_controller', 'joint_coxa_rr_controller', 'joint_femur_rr_controller', 'joint_tibia_rr_controller']
  
  for controller in robot_controllers_active:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
                emulate_tty=True,
            )
        ]


  delay_joint_state_broadcaster_spawner_after_ros2_control_node = (
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=control_node,
                    on_start=[
                        TimerAction(
                            period=2.0,
                            actions=[joint_state_broadcaster_spawner,],
                        ),
                    ],
                )
            )
        )

  delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
  for controller in robot_controller_spawners:
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[
                        TimerAction(
                            period=2.0,
                            actions=[controller],
                        ),
                    ],
                )
            )
        ]

# ****************************
#               End          *
# ****************************

  control_node_start = []
  control_node_start.append(robot_state_pub_node)
  control_node_start.append(control_node)
  control_node_start.append(rviz_node)
  control_node_start.append(delay_joint_state_broadcaster_spawner_after_ros2_control_node)
  control_node_start.append(container_parametrs)
  


  return LaunchDescription(declared_arguments +
                            control_node_start +
                            delay_robot_controller_spawners_after_joint_state_broadcaster_spawner 
                            )      
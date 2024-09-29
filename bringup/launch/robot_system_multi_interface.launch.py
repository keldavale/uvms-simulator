from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os, yaml, xacro, copy

 
class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True
    
def modify_controller_config(config_path,new_config_path,robot_count:int=1)->None:
        with open(config_path,'r') as file:
            controller_param = yaml.load(file,yaml.SafeLoader)
        new_param = copy.deepcopy(controller_param)
        
        for i in range(2, robot_count + 1):
            agent_name = f'bluerov_alpha_{i}'
            prefix = f'robot_{i}_'
            base_link = f'{prefix}base_link'
            IOs = f'{prefix}IOs'

            # Add agent to the uvms_controller parameters
            new_param['uvms_controller']['ros__parameters']['agents'].append(agent_name)

            # Add agent-specific parameters under uvms_controller
            new_param['uvms_controller']['ros__parameters'][agent_name] = {
                'prefix': prefix,
                'base_TF_translation': [0.140, 0.000, -0.120],
                'base_TF_rotation': [3.142, 0.000, 0.000]
            }

            # Add IMU sensor broadcaster
            imu_broadcaster_name = f'imu_broadcaster_{i}'
            new_param['controller_manager']['ros__parameters'][imu_broadcaster_name] = {
                'type': 'imu_sensor_broadcaster/IMUSensorBroadcaster'
            }

            fts_broadcaster_name = f'fts_broadcaster_{i}'
            new_param['controller_manager']['ros__parameters'][fts_broadcaster_name] = {
                'type': 'force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster'
            }

            new_param[imu_broadcaster_name] = {'ros__parameters': {
                    'frame_id': base_link,
                    'sensor_name': IOs
                }
            }

            new_param[fts_broadcaster_name] = {'ros__parameters': {
                'frame_id': base_link,
                'interface_names': {
                    'force': {
                        'x': f'{IOs}/force.x',
                        'y': f'{IOs}/force.y',
                        'z': f'{IOs}/force.z'
                        },
                    'torque': {
                        'x': f'{IOs}/torque.x',
                        'y': f'{IOs}/torque.y',
                        'z': f'{IOs}/torque.z'
                        }
                    }
                }
            }


        new_controller_param = copy.deepcopy(new_param)
        with open(new_config_path,'w') as file:
            yaml.dump(new_controller_param,file,Dumper=NoAliasDumper)



def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='alpha',
            description="Prefix of the joint names, useful for multi-robot setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Start robot with device port to hardware.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "state_update_frequency",
            default_value="200",
            description="The frequency (Hz) at which the driver updates the state of the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_count",
            default_value="1",
            description="Spawn with n numbers of robot agents",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="forward_current_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

def launch_setup(context, *args, **kwargs):
    # Resolve LaunchConfigurations
    prefix = LaunchConfiguration("prefix").perform(context)
    use_mock_hardware = LaunchConfiguration("use_mock_hardware").perform(context)
    serial_port = LaunchConfiguration("serial_port").perform(context)
    state_update_frequency = LaunchConfiguration("state_update_frequency").perform(context)
    robot_controller = LaunchConfiguration("robot_controller").perform(context)
    gui = LaunchConfiguration("gui").perform(context)
    robot_count = int(LaunchConfiguration("robot_count").perform(context))

    # Define the robot description command
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_control_blue_reach_5"),
                    "xacro",
                    "robot_system_multi_interface.urdf.xacro",
                ]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "serial_port:=",
            serial_port,
            " ",
            "state_update_frequency:=",
            state_update_frequency,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "robot_count:=",
            TextSubstitution(text=str(robot_count)),
        ]
    )

    robot_description = {"robot_description": robot_description_content}
    robot_controllers_read = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_blue_reach_5"),
            "config",
            "robot_multi_interface_forward_controllers.yaml",
        ]
    )
    robot_controllers_modified = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_blue_reach_5"),
            "config",
            "robot_multi_interface_forward_controllers_modified.yaml",
        ]
    )
    # resolve PathJoinSubstitution to a string
    robot_controllers_read_str = str(robot_controllers_read.perform(context))
    robot_controllers_modified_str = str(robot_controllers_modified.perform(context))
    modify_controller_config(robot_controllers_read_str, robot_controllers_modified_str, robot_count)

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ros2_control_blue_reach_5"), "rviz", "rviz.rviz"]
    )

    # Nodes Definitions
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers_modified, robot_description],
        output="both",
    )

    # Spawner Nodes
    spawner_nodes = []

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    spawner_nodes.append(joint_state_broadcaster_spawner)

    # UVMS Controller Spawner (if using mock hardware)
    uvms_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["uvms_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(use_mock_hardware),
    )
    spawner_nodes.append(uvms_spawner)

    # Robot Controller Spawner (unless using mock hardware)
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller, "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(use_mock_hardware),
    )
    spawner_nodes.append(robot_controller_spawner)

    # Spawn fts and imu broadcasters for each robot
    for i in range(1, robot_count + 1):
        fts_broadcaster_name = f'fts_broadcaster_{i}'
        imu_broadcaster_name = f'imu_broadcaster_{i}'

        # FTS Spawner
        fts_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[fts_broadcaster_name, "--controller-manager", "/controller_manager"],
        )
        spawner_nodes.append(fts_spawner)

        # IMU Spawner
        imu_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[imu_broadcaster_name, "--controller-manager", "/controller_manager"],
        )
        spawner_nodes.append(imu_spawner)

    # Delay RViz start after `joint_state_broadcaster_spawner`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster_spawner`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # Define other nodes if needed
    run_plotjuggler = ExecuteProcess(
        cmd=['ros2', 'run', 'plotjuggler', 'plotjuggler > /dev/null 2>&1'],
        output='screen',
        shell=True
    )

    # Collect all nodes
    nodes = [
        run_plotjuggler,
        control_node,
        robot_state_pub_node,
        # Removed individual additions of spawners
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ] + spawner_nodes

    return nodes

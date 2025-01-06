from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os, yaml, xacro, copy
import ast
 
class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True

def rviz_file_configure(robot_prefixes, robot_base_links, ix, rviz_config_path,new_rviz_config_path)->None:
    # Load the RViz configuration file
    with open(rviz_config_path,'r') as file:
        rviz_config = yaml.load(file,yaml.SafeLoader)
    new_rviz_config = copy.deepcopy(rviz_config)

    rviz_view_configure(robot_prefixes, robot_base_links, new_rviz_config)
    rviz_axes_configure(robot_prefixes, new_rviz_config)
    add_wrench_entries(ix, new_rviz_config)
    with open(new_rviz_config_path,'w') as file:
        yaml.dump(new_rviz_config,file,Dumper=NoAliasDumper)

def rviz_axes_configure(robot_prefixes, rviz_config):
    for prefix in robot_prefixes:
        added_axes = {'Class': 'rviz_default_plugins/Axes',
        'Enabled': True,
        'Length': '0.10000000149011612',
        'Name': f'{prefix}vehicle_base',
        'Radius': '0.009999999776482582',
        'Reference Frame': f"{prefix}vehicle_base",
        'Value': True}
        rviz_config['Visualization Manager']['Displays'].append(added_axes)
        
        for i in range(5):
            added_axes = {'Class': 'rviz_default_plugins/Axes',
            'Enabled': True,
            'Length': '0.10000000149011612',
            'Name': f'{prefix}joint_{i}',
            'Radius': '0.009999999776482582',
            'Reference Frame': f"{prefix}joint_{i}",
            'Value': True}
            rviz_config['Visualization Manager']['Displays'].append(added_axes)

def rviz_view_configure(robot_prefixes, robot_base_links, rviz_config):
    rviz_config['Visualization Manager']['Views']['Saved'] = []
    original_view = {
        'Class': 'rviz_default_plugins/Orbit',
        'Distance': 8.755668640136719,
        'Enable Stereo Rendering': {
          'Stereo Eye Separation': 0.05999999865889549,
          'Stereo Focal Distance': 1,
          'Swap Stereo Eyes': False,
          'Value': False},
        'Focal Point': {
          'X': 0,
          'Y': 0,
          'Z': 0},
        'Focal Shape Fixed Size': True,
        'Focal Shape Size': 0.05000000074505806,
        'Invert Z Axis': False,
        'Name': 'NED Origin',
        'Near Clip Distance': 0.009999999776482582,
        'Pitch': 0.44020360708236694,
        'Target Frame': 'base_link',
        'Value': 'Orbit (rviz)',
        'Yaw': 3.3494811058044434
    }
    rviz_config['Visualization Manager']['Views']['Saved'].append(original_view)
    for i, prefix in enumerate(robot_prefixes):
        new_view = original_view.copy()
        new_view['Name'] = f'{prefix} view'
        new_view['Target Frame'] = robot_base_links[i]
        rviz_config['Visualization Manager']['Views']['Saved'].append(new_view)


def add_wrench_entries(ix, rviz_config):
    # The existing wrench configuration you want to replicate
    original_wrench = {
        'Accept NaN Values': False,
        'Alpha': 1,
        'Arrow Width': 0.3,
        'Class': 'rviz_default_plugins/Wrench',
        'Enabled': True,
        'Force Arrow Scale': 0.7,
        'Force Color': '204; 51; 51',
        'History Length': 1,
        'Name': 'Wrench',
        'Torque Arrow Scale': 0.7,
        'Torque Color': '204; 204; 51',
        'Value': True
    }

    # Add new Wrench entries with the incremented index in the 'Value' field
    for i in ix:
        new_wrench = original_wrench.copy()
        new_wrench['Name'] = f'robot_Wrench_{i}'
        new_wrench['Topic'] = {
            'Depth': 5,
            'Durability Policy': 'Volatile',
            'Filter size': 10,
            'History Policy': 'Keep Last',
            'Reliability Policy': 'Reliable',
            'Value': f'/fts_broadcaster_{i}/wrench'
        }
        rviz_config['Visualization Manager']['Displays'].append(new_wrench)

def modify_controller_config(any_real_hardware, config_path,new_config_path,sim_robot_count:int=1)->None:
        with open(config_path,'r') as file:
            controller_param = yaml.load(file,yaml.SafeLoader)
        new_param = copy.deepcopy(controller_param)

        controller_descriptor = new_param['uvms_controller']['ros__parameters']

        dof_efforts = [
                effort 
                for joint in controller_descriptor['joints'] 
                if joint in controller_descriptor 
                for effort in controller_descriptor[joint].get('effort_command_interface', [])
            ]

        ix = []
        robot_prefixes = []
        robot_base_links = []

        new_param['uvms_controller']['ros__parameters']['agents'] = []
        if any_real_hardware:
            i = 'real'
            new_param, robot_prefixes, robot_base_links, ix = add_uvms_model_control(new_param, i, robot_prefixes, robot_base_links, ix)
            
        for i in range(1, sim_robot_count + 1):
            new_param, robot_prefixes, robot_base_links, ix = add_uvms_model_control(new_param, i, robot_prefixes, robot_base_links, ix)

        with open(new_config_path,'w') as file:
            yaml.dump(new_param,file,Dumper=NoAliasDumper)
        return robot_prefixes, robot_base_links, ix, dof_efforts

def add_uvms_model_control(new_param, i, robot_prefixes, robot_base_links, ix):
    ix.append(i)
    agent_name = f'bluerov_alpha_{i}'
    prefix = f'robot_{i}_'

    robot_prefixes.append(prefix)

    base_link = f'{prefix}base_link'
    robot_base_links.append(base_link)

    IOs = f'{prefix}IOs'
        # Add agent to the uvms_controller parameters
    new_param['uvms_controller']['ros__parameters']['agents'].append(agent_name)

    # Add agent-specific parameters under uvms_controller
    new_param['uvms_controller']['ros__parameters'][agent_name] = {
        'prefix': prefix,
        'base_TF_translation': [0.140, 0.000, -0.120],
        'base_TF_rotation': [3.142, 0.000, 0.000],
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
    return new_param, robot_prefixes, robot_base_links, ix

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
            "use_manipulator_hardware",
            default_value="false",
            description="Start simulation with a real manipulator hardware in the loop",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_vehicle_hardware",
            default_value="false",
            description="Start simulation with a real vehicle hardware in the loop",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_robot_count",
            default_value="1",
            description="Spawn with n numbers of robot agents",
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
    use_manipulator_hardware = LaunchConfiguration("use_manipulator_hardware").perform(context)
    use_vehicle_hardware = LaunchConfiguration("use_vehicle_hardware").perform(context)
    serial_port = LaunchConfiguration("serial_port").perform(context)
    state_update_frequency = LaunchConfiguration("state_update_frequency").perform(context)
    gui = LaunchConfiguration("gui").perform(context)
    sim_robot_count = int(LaunchConfiguration("sim_robot_count").perform(context))

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
            "use_manipulator_hardware:=",
            use_manipulator_hardware,
            " ",
            "use_vehicle_hardware:=",
            use_vehicle_hardware,
            " ",
            "sim_robot_count:=",
            TextSubstitution(text=str(sim_robot_count)),
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
    robot_controllers_read_file = str(robot_controllers_read.perform(context))
    robot_controllers_modified_file = str(robot_controllers_modified.perform(context))
    any_real_hardware = use_manipulator_hardware or use_vehicle_hardware
    any_real_hardware_bool = IfCondition(any_real_hardware).evaluate(context)
    robot_prefixes, robot_base_links, ix, dof_efforts = modify_controller_config(any_real_hardware_bool, robot_controllers_read_file, robot_controllers_modified_file, sim_robot_count)

    rviz_config_read = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_blue_reach_5"),
            "rviz",
            "rviz.rviz",
        ]
    )
    rviz_config_modified = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_blue_reach_5"),
            "rviz",
            "rviz_modified.rviz",
        ]
    )
    # resolve PathJoinSubstitution to a string
    rviz_config_read_file = str(rviz_config_read.perform(context))
    rviz_config_modified_file = str(rviz_config_modified.perform(context))
    
    rviz_file_configure(robot_prefixes, robot_base_links, ix, rviz_config_read_file, rviz_config_modified_file)

    mavros_config = PathJoinSubstitution(
            [
                FindPackageShare("ros2_control_blue_reach_5"),
                "mavros",
                "mavros.yaml",
            ]
        )
    mavros_config_file = str(mavros_config.perform(context))
    uv_hardware_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'mavros', 'mavros_node',
            '--ros-args', '--params-file', f'{mavros_config_file}'
        ],
        shell=False,
        condition=IfCondition(use_vehicle_hardware)
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
        arguments=["-d", rviz_config_modified],
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
        arguments=["uvms_controller", "--controller-manager", "/controller_manager"]
    )
    spawner_nodes.append(uvms_spawner)
    
    # Spawn fts and imu broadcasters for each robot
    for i in ix:
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
            target_action=fts_spawner,
            on_exit=[rviz_node],
        )
    )

    # Define other nodes if needed
    run_plotjuggler = ExecuteProcess(
        cmd=['ros2', 'run', 'plotjuggler', 'plotjuggler > /dev/null 2>&1'],
        output='screen',
        shell=True,
        condition=IfCondition(any_real_hardware)
    )
    
    mouse_control = Node(
        package='simlab',
        executable='mouse_node_effort',
        name='space_mouse_control',
        parameters=[{
            'no_robot': len(robot_prefixes) ,
            'no_efforts': len(dof_efforts)
        }]
    )

    sensorPy_node = Node(
        package='simlab',
        executable='sensor_py_node',
        condition=IfCondition(any_real_hardware)
    )

    # Collect all nodes
    nodes = [
        uv_hardware_node,
        sensorPy_node,
        # mouse_control,
        run_plotjuggler,
        control_node,
        robot_state_pub_node,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ] + spawner_nodes

    return nodes

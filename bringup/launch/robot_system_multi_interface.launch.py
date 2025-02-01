from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, OpaqueFunction, TimerAction, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml, copy
import random

class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True

def rviz_file_configure(use_vehicle_hardware, use_manipulator_hardware, robot_prefixes, robot_base_links, ix, rviz_config_path,new_rviz_config_path)->None:
    # Load the RViz configuration file
    with open(rviz_config_path,'r') as file:
        rviz_config = yaml.load(file,yaml.SafeLoader)
    new_rviz_config = copy.deepcopy(rviz_config)

    rviz_view_configure(robot_prefixes, robot_base_links, new_rviz_config)
    rviz_states_axes_configure(robot_prefixes, new_rviz_config)
    rviz_robots_path_configure(robot_prefixes, new_rviz_config)
    if use_vehicle_hardware:
        imu_display("Imu Sensor", "/mavros/imu/data", new_rviz_config, False)
        rviz_axes_display('imu_frame', "imu_link", new_rviz_config, 0.3, 0.02, False)
        rviz_axes_display('dvl_frame', "dvl_link", new_rviz_config, 0.1, 0.01, False)

    add_wrench_entries(ix, new_rviz_config)
    with open(new_rviz_config_path,'w') as file:
        yaml.dump(new_rviz_config,file,Dumper=NoAliasDumper)


def rviz_path_display(name, topic, rviz_config, color, enabled):
    path_config = {
        "Alpha": 1,
        "Buffer Length": 1,
        "Class": "rviz_default_plugins/Path",
        "Color": color,
        "Enabled": enabled,
        "Head Diameter": 0.3,
        "Head Length": 0.2,
        "Length": 0.3,
        "Line Style": "Lines",
        "Line Width": 0.03,
        "Name": name,
        "Offset": {
            "X": 0,
            "Y": 0,
            "Z": 0
        },
        "Pose Color": "255; 85; 255",
        "Pose Style": "None",
        "Radius": 0.03,
        "Shaft Diameter": 0.1,
        "Shaft Length": 0.1,
        "Topic": {
            "Depth": 5,
            "Durability Policy": "Volatile",
            "Filter size": 40,
            "History Policy": "Keep Last",
            "Reliability Policy": "Reliable",
            "Value": topic
        },
        "Value": True
    }
    rviz_config['Visualization Manager']['Displays'].append(path_config)


def rviz_axes_display(name, reference_frame, rviz_config, length, radius, enabled):
    added_axes = {'Class': 'rviz_default_plugins/Axes',
        'Enabled': enabled,
        'Length': str(length),
        'Name': name,
        'Radius': str(radius),
        'Reference Frame': reference_frame,
        'Value': True}
    rviz_config['Visualization Manager']['Displays'].append(added_axes)

def imu_display(name, topic,rviz_config, enabled):
    imu_config = {
            "Acceleration properties": {
                "Acc. vector alpha": 0.10000000149011612,
                "Acc. vector color": "255; 0; 0",
                "Acc. vector scale": 0.05000000074505806,
                "Derotate acceleration": True,
                "Enable acceleration": False
            },
            "Axes properties": {
                "Axes scale": 0.2,
                "Enable axes": True
            },
            "Box properties": {
                "Box alpha": 1,
                "Box color": "255; 0; 0",
                "Enable box": False,
                "x_scale": 1,
                "y_scale": 1,
                "z_scale": 1
            },
            "Class": "rviz_imu_plugin/Imu",
            "Enabled": enabled,
            "Name": name,
            "Topic": {
                "Depth": 13,
                "Durability Policy": "Volatile",
                "Filter size": 10,
                "History Policy": "Keep Last",
                "Reliability Policy": "Best Effort",
                "Value": topic
            },
            "Value": True,
            "fixed_frame_orientation": True
        }
    rviz_config['Visualization Manager']['Displays'].append(imu_config)

def generate_random_color(path_type=None, default=False):
    """
    Generates a random color in the "R; G; B" string format.
    
    Returns:
    - str: Random color as "R; G; B".
    """
    if default and path_type=='ref_path':
        default_path_color = "248; 228; 92"       # Yellow for the first robot's Path
        return default_path_color
    if default and path_type=='robot_path': 
        default_traj_color = "224;27;36"        # Red for the first robot's TrajectoryPath
        return default_traj_color
        
    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)
    return f"{r}; {g}; {b}"

def rviz_robots_path_configure(robot_prefixes, rviz_config):
    enabled = True
    for idx, prefix in enumerate(robot_prefixes):
        if idx == 0:
            path_color = generate_random_color('ref_path', True)
            traj_color = generate_random_color('robot_path', True)
        else:
            path_color = generate_random_color()
            traj_color = generate_random_color()
        rviz_path_display(f"{prefix}/Path", f"/{prefix}Path", rviz_config, path_color, enabled)
        rviz_path_display(f"{prefix}/TrajectoryPath", f"/{prefix}TrajectoryPath", rviz_config, traj_color, enabled)

def rviz_states_axes_configure(robot_prefixes, rviz_config):
    for prefix in robot_prefixes:
        base_link = f'{prefix}base_link'
        robot_map_frame = f'{prefix}map'
        rviz_axes_display(base_link, base_link, rviz_config, 0.1, 0.01, True)
        rviz_axes_display(robot_map_frame, f'{prefix}map', rviz_config, 0.1, 0.01, True)
        for i in range(5):
            rviz_axes_display(f'{prefix}joint_{i}', f"{prefix}joint_{i}", rviz_config, 0.1, 0.01, True)


    
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
        'Name': 'World Origin',
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

def modify_controller_config(use_vehicle_hardware, use_manipulator_hardware, config_path,new_config_path,sim_robot_count:int=1):
        
        if not use_vehicle_hardware and not use_manipulator_hardware and sim_robot_count==0:
            raise Exception("Invalid configuration: No robots specified. Enable either vehicle or manipulator hardware, or specify at least one simulation robot.")

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

        if use_manipulator_hardware or use_vehicle_hardware:
            i = 'real'
            new_param, robot_prefixes, robot_base_links, ix = add_uvms_model_control(use_vehicle_hardware, use_manipulator_hardware, new_param, i, robot_prefixes, robot_base_links, ix)

        for i in range(1, sim_robot_count + 1):
            new_param, robot_prefixes, robot_base_links, ix = add_uvms_model_control(use_vehicle_hardware, use_manipulator_hardware, new_param, i, robot_prefixes, robot_base_links, ix)

        with open(new_config_path,'w') as file:
            yaml.dump(new_param,file,Dumper=NoAliasDumper)
        return robot_prefixes, robot_base_links, ix, dof_efforts

def add_uvms_model_control(use_vehicle_hardware, use_manipulator_hardware, new_param, i, robot_prefixes, robot_base_links, ix):
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

    fts_broadcaster_name = f'fts_broadcaster_{i}'
    new_param['controller_manager']['ros__parameters'][fts_broadcaster_name] = {
        'type': 'force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster'
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
            "task",
            default_value="manual",
            description="Start simulation with a task",
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
    task = LaunchConfiguration("task").perform(context)
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
            "task:=",
            task,
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

    use_manipulator_hardware_bool = IfCondition(use_manipulator_hardware).evaluate(context)
    use_vehicle_hardware_bool = IfCondition(use_vehicle_hardware).evaluate(context)

    robot_prefixes, robot_base_links, ix, dof_efforts = modify_controller_config(use_vehicle_hardware_bool,
                                                                                 use_manipulator_hardware_bool,
                                                                                  robot_controllers_read_file,
                                                                                    robot_controllers_modified_file,
                                                                                      sim_robot_count)

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
    
    rviz_file_configure(use_vehicle_hardware_bool, use_manipulator_hardware_bool,robot_prefixes, robot_base_links, ix, rviz_config_read_file, rviz_config_modified_file)

    mavros_config = PathJoinSubstitution(
            [
                FindPackageShare("ros2_control_blue_reach_5"),
                "config",
                "mavros.yaml",
            ]
        )
    mavros_config_file = str(mavros_config.perform(context))
    mavros_node = ExecuteProcess(
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

        # FTS Spawner
        fts_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[fts_broadcaster_name, "--controller-manager", "/controller_manager"],
        )
        spawner_nodes.append(fts_spawner)

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
        shell=True
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

    coverage_node = Node(
        package='simlab',
        executable='coverage_node',
        parameters=[{
            'robots_prefix': robot_prefixes,
            'no_robot': len(robot_prefixes) ,
            'no_efforts': len(dof_efforts)
        }]
    )

    

    kf_node = Node(
        package='bluerov_kalmanfilter',
        executable='kf_node',
        condition=IfCondition(use_vehicle_hardware),
        parameters=[{
            'use_pressure': False,
            'dvl_source': 'ros_hil_simulator'
        }]
    )

    shape_formation_node = Node(
        package='simlab',
        executable='shape_formation_node',
        parameters=[{
            'robots_prefix': robot_prefixes,
            'no_robot': len(robot_prefixes) ,
            'no_efforts': len(dof_efforts)
        }]
    )
    
    station_node = Node(
        package='simlab',
        executable='station_node',
        parameters=[{
            'robots_prefix': robot_prefixes,
            'no_robot': len(robot_prefixes) ,
            'no_efforts': len(dof_efforts)
        }]
    )

    uvms_ops_node = Node(
        package='simlab',
        executable='uvms_ops_node',
        parameters=[{
            'robots_prefix': robot_prefixes,
            'no_robot': len(robot_prefixes) ,
            'no_efforts': len(dof_efforts)
        }]
    )

    ik_solve_node = Node(
        package='simlab',
        executable='ik_solve_node',
        parameters=[{
            'robots_prefix': robot_prefixes,
            'no_robot': len(robot_prefixes) ,
            'no_efforts': len(dof_efforts)
        }]
    )

    

    if task == 'manual':
        mode = mouse_control
    elif task == 'coverage':
        mode = coverage_node
    elif task == 'station':
        mode = station_node
    elif task == 'formation':
        mode = shape_formation_node
    elif task == 'ops':
        mode = uvms_ops_node
    elif task == 'ik':
        mode = ik_solve_node

  # Define the simulator actions
    simulator_actions = [
        control_node,
        kf_node,
        mode,
        run_plotjuggler,
        robot_state_pub_node,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ] + spawner_nodes
    
    # Define delayed simulator_agent (15-second delay) conditioned on use_vehicle_hardware=True
    simulator_agent_delayed = TimerAction(
        period=15.0,
        actions=simulator_actions,
        condition=IfCondition(use_vehicle_hardware)
    )
    
    # Define immediate simulator_agent (no delay) conditioned on use_vehicle_hardware=False
    simulator_agent_immediate = GroupAction(
        actions=simulator_actions,
        condition=UnlessCondition(use_vehicle_hardware)
    )
    
    # Launch nodes
    nodes = [
        mavros_node,
        simulator_agent_delayed,
        simulator_agent_immediate
    ]
    
    return nodes
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    hexapod_nodes_path = get_package_share_path('hexapod_nodes')
    model_path = hexapod_nodes_path / 'urdf/hexapod.urdf.xacro'
    default_rviz_config_path = hexapod_nodes_path / 'rviz/urdf.rviz'

    # Launch args
    joint_gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'], 
                                        description='Flag to enable joint_state_publisher_gui')
    rviz_arg = DeclareLaunchArgument(name='rviz', default_value='true', choices=['true', 'false'], 
                                        description='Flag to enable rviz')
    rviz_config = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path), 
                                        description='Path to RVIZ config file')

    use_rviz = LaunchConfiguration('rviz')
    use_gui = LaunchConfiguration('gui')

    robot_description = ParameterValue(Command(['xacro ', str(model_path)]), value_type=str)

    # RVIZ nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description
        }]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        # There is no proper support for multiple launch conditions, but this workaround addresses that.
        condition=IfCondition(
            PythonExpression(["not '", use_gui, "' and not '", use_rviz, "'"])
        )
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(
            PythonExpression(["'", use_gui, "' and not '", use_rviz, "'"])
        )
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )



    # LEG 1 nodes
    leg1_movement_controller_node = Node(
        package='hexapod_nodes',
        namespace='leg_1',
        executable='leg_movement_controller',
        name='leg_movement_controller',
        parameters=[{
            "leg_id": 1
        }]
    )
    leg1_servo_controller_node = Node(
        package='hexapod_nodes',
        namespace='leg_1',
        executable='leg_servo_controller',
        name='leg_servo_controller',
        parameters=[{
            "leg_id": 1
        }]
    )

    return LaunchDescription([
        joint_gui_arg,
        rviz_arg,
        rviz_config,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,

        leg1_movement_controller_node,
        leg1_servo_controller_node
    ])

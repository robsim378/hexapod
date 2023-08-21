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
    rviz_arg = DeclareLaunchArgument(name='rviz', default_value='true', choices=['true', 'false'], 
                                        description='Flag to enable rviz')
    rviz_config = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path), 
                                        description='Path to RVIZ config file')

    model_arg = DeclareLaunchArgument(name='model', default_value=str(model_path),
                                        description="Path to URDF model")

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                        value_type=str)

    # RVIZ nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description
        }]
    )
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('rviz'))
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
    leg_1_movement_controller_node = Node(
        package='hexapod_nodes',
        namespace='leg_1',
        executable='leg_movement_controller',
        name='leg_movement_controller',
        parameters=[{
            "leg_id": 1
        }]
    )
    leg_1_servo_controller_node = Node(
        package='hexapod_nodes',
        namespace='leg_1',
        executable='leg_servo_controller',
        name='leg_servo_controller',
        parameters=[{
            "leg_id": 1
        }]
    )
    leg_1_state_broadcaster_node = Node(
        package='hexapod_nodes',
        namespace='leg_1',
        executable='leg_state_broadcaster',
        name='leg_state_broadcaster',
        parameters=[{
            "leg_id": 1
        }]
    )

    return LaunchDescription([
        rviz_arg,
        rviz_config,
        model_arg,
        #joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,

        leg_1_movement_controller_node,
        leg_1_servo_controller_node,
        leg_1_state_broadcaster_node,
    ])

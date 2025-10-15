from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # List of original nodes
        # - move_panda.launch

        # move_panda.launch nodes:
        # - Main
        # - Vizualization

        Node(
            package='hiro_collision_avoidance_ros2',
            executable='Main',
            name='Main',
            output='screen',
            parameters=[{'avoidance_type': 'hiro', 'movement_type': 'static'}],
        ),


        # Simulation:
        # - robot.launch

        # robot.launch:
        # - simulation.launch
        # - panda.launch

        # simulation.launch:
        # - robot_description
        # - gazebo empty_world.launch
        # - position and velocity controllers
        # - robot_state_publisher
        # - rviz

        # panda.launch:
        # franka_control.launch
        # panda_joint_controllers.yaml
        # rviz

        # New Plan
        # - Need to get the joint states from the franka_ros2 (custom node)
        # - Send the desired velocity and position controls to the franka_ros2 (custom node)
        # - No gazebo
        # - No rviz

        
    ])

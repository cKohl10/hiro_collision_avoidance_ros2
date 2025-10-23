# ROS2 Collision Avoidance for FR3
#### *Human Interaction and Robotics Group (HIRO)*

## Run the Controller

```
param_file = PathJoinSubstitution([
    FindPackageShare('gentact_ros_tools'),
    'config',
    'fr3.yaml',
])

Node(
    package='hiro_collision_avoidance_ros2',
    executable='Main',
    name='avoidance_controller',
    output='screen',
    parameters=[param_file, {
        'avoidance_type': avoidance_type, 
        'movement_type': movement_type, 
        'robot_description': robot_description}],
)
```

You will need to include a config file that contains the joints to be solved for. Create a new folder in your package called `config` and create a file called `fr3.yaml`. Copy and paste the following code in this file
```
/**:
  ros__parameters:
    namespace: ""
    arm_id: "fr3"
    base_link: "fr3_link1"
    ee_link: "fr3_link7"
```

## Controller Structure
The avoidance controller subscribes to `/joint_states` and publishes velocity commands on `/fr3_joint_velocity_controller/command`. 
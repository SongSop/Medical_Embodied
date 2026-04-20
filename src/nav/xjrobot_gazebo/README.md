# xjrobot_gazebo

Gazebo simulation package for `xjrobot`, refactored to follow the `linorobot2_gazebo` layout.

## Worlds

- `worlds/xjrobot/`: custom xjrobot environments (`myworld2`, `simple`, `simple_env`) in `.sdf` format only
- `worlds/lino/`: retained linorobot environments (`playground`, `turtlebot3_world`)

## Launch

```bash
ros2 launch xjrobot_gazebo gazebo.launch.py
```

Examples:

```bash
# xjrobot custom world
ros2 launch xjrobot_gazebo gazebo.launch.py world_name:=xjrobot/myworld2.sdf

# lino world
ros2 launch xjrobot_gazebo gazebo.launch.py world_name:=lino/playground.sdf
```

# Add Obstacle Costmap Plugin Layer
1. Adds a rectangular costmap window of a configurable size as an obstacle of LETHAL_OBSTACLE cost (254).
2. The plugin can be added to the nav2 params file in the local and/or global costmap params as follows:
```yaml
  add_obstacle_layer:
    plugin: nav2_costmap_2d::AddObstacleLayer
    enabled: True
    update_window_height_m: 1.5
    update_window_width_m: 0.5
```
3. The obstacles are published as a geometry_msgs/msg/Point message on the "/summit/sensor_obstacles" in the map frame.
```bash
ros2 topic pub /summit/sensor_obstacles geometry_msgs/msg/Point "x: 1.0
y: -4.5
z: 0.0"
```
4. For having the plugin enabled we have to checkout the `humble-costmap-update` branch of this `navigation2` repo and then build and source only the `nav2_costmap_2d`package if the `navigation2` is installed via `apt`.
```bash
colcon build --symlink-install --packages-select nav2_costmap_2d
. install/setup.bash
```

编译：
colcon build

source:
source install/setup.bash

运行：
ros2 launch robot_navigation navigation.launch.py

配置地图：
ros2 lifecycle set /map_server configure

激活地图：
ros2 lifecycle set /map_server activate


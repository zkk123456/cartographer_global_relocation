开启切换地图服务的节点：
source install/setup.bash
ros2 run navigation_server change_map_server

请求切换地图：
ros2 service call /change_map beefast_interfaces/srv/ChangeMap "{file_path: '/path/to/map.pbstream', initial_pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

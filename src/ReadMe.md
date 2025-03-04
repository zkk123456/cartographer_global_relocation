# 目录：

|-cartographer_ws

 |-abseil-cpp 依赖的库  
 
​	|-cartographer_ros_msgs  建图格式定义

​	|-cartographer 建图核心算法，依赖cartographer_ros_msgs

 |-cartographer_ros 建图ros接口，依赖 cartographer_ros_msgs和cartographer


# 编译：
  首先手动编译依赖的库abseil;
      sudo sh ./src/abseil-cpp/install_abseil.sh
      
  编译cartographer:
   colcon build --merge-install --cmake-args -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON --packages-select cartographer_ros_msgs
   
   colcon build --merge-install --cmake-args -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON --packages-select cartographer
   
   colcon build --merge-install --cmake-args -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON --packages-select cartographer_ros
   
# 使用(需提前开启仿真环境或打开真实环境，才能使用建图)
  source install/local_setup.sh
  
  ros2 launch cartographer_ros my_carto_launch.py
  

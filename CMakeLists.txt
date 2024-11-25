cmake_minimum_required(VERSION 3.0.2)
project(restricted_areas_plugin)

# 查找Qt5依赖项
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# 查找ROS依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  dynamic_reconfigure
  rviz
  geometry_msgs
  tf2_ros
  pluginlib
  std_srvs
  rospack
)

find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# 生成动态配置API
generate_dynamic_reconfigure_options(
  config/RestrictedAreas.cfg
  config/CostmapProhibitionLayer.cfg  # 代价地图插件
)

# 设置catkin构建信息
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES costmap_prohibition_layer ${PROJECT_NAME}_rviz
  CATKIN_DEPENDS roscpp costmap_2d dynamic_reconfigure rviz geometry_msgs tf2_ros std_srvs rospack
  DEPENDS
)

# 包含头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# 构建 area_info 库
add_library(${PROJECT_NAME}_common
  include/${PROJECT_NAME}/area_info.h
)
set_target_properties(${PROJECT_NAME}_common PROPERTIES LINKER_LANGUAGE CXX)

# 构建costmap插件库
add_library(${PROJECT_NAME}_costmap
  src/restricted_areas_layer.cpp
)
add_dependencies(${PROJECT_NAME}_costmap
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_common
)
target_link_libraries(${PROJECT_NAME}_costmap
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROJECT_NAME}_common
)

# 生成Qt MOC文件
qt5_wrap_cpp(MOC_FILES
  include/${PROJECT_NAME}/restricted_areas_display.h
  include/${PROJECT_NAME}/restricted_areas_tool.h
)

# 构建RViz插件库
add_library(${PROJECT_NAME}_rviz
  src/restricted_areas_display.cpp
  src/restricted_areas_tool.cpp
  ${MOC_FILES}
)
target_link_libraries(${PROJECT_NAME}_rviz
  ${catkin_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROJECT_NAME}_common
)

# 安装目标
install(TARGETS ${PROJECT_NAME}_costmap ${PROJECT_NAME}_rviz
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES costmap_plugins.xml rviz_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 安装图标文件
install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.svg"
)
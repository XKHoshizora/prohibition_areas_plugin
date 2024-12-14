cmake_minimum_required(VERSION 3.0.2)
project(prohibition_areas_plugin)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

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
  xmlrpcpp
  message_generation
  message_runtime
)

# 查找其他依赖项
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# 启用 Qt 自动生成
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Qt资源文件编译
set(RESOURCES_FILES
  resources/prohibition_areas_plugin.qrc
)

# 为RViz插件单独设置Qt处理
qt5_wrap_cpp(MOC_FILES
  include/${PROJECT_NAME}/prohibition_areas_tool/edit_points_frame.h
  include/${PROJECT_NAME}/prohibition_areas_tool/prohibition_areas_display.h
  include/${PROJECT_NAME}/prohibition_areas_tool/prohibition_areas_tool.h
)

# 生成动态配置API
generate_dynamic_reconfigure_options(
  config/ProhibitionAreasLayer.cfg  # 代价地图插件配置
)

add_message_files(
  FILES
  ProhibitionArea.msg
  ProhibitionAreas.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

# 包含头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

# 设置catkin构建信息
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES prohibition_areas_layer prohibition_areas_tool
  CATKIN_DEPENDS roscpp costmap_2d dynamic_reconfigure rviz geometry_msgs tf2_ros std_srvs rospack xmlrpcpp message_generation message_runtime
  DEPENDS YAML_CPP Qt5Core Qt5Widgets
)

# 构建 Costmap 插件库
add_library(prohibition_areas_layer
  src/prohibition_areas_layer/prohibition_areas_layer.cpp
)
add_dependencies(prohibition_areas_layer
  ${PROJECT_NAME}_gencfg
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(prohibition_areas_layer
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# 构建 RViz 插件库
add_library(prohibition_areas_tool
  ${MOC_FILES}
  src/prohibition_areas_tool/prohibition_areas_tool.cpp
  src/prohibition_areas_tool/edit_points_frame.cpp
  src/prohibition_areas_tool/prohibition_areas_display.cpp
  src/prohibition_areas_tool/prohibition_areas_saver.cpp
  ${RESOURCES_FILES}
)
add_dependencies(prohibition_areas_tool
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(prohibition_areas_tool
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
)

# 添加编译选项
set_target_properties(prohibition_areas_tool PROPERTIES
  AUTOMOC ON
  POSITION_INDEPENDENT_CODE ON
)

# 确保包含当前目录
target_include_directories(prohibition_areas_tool PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 安装目标
install(TARGETS
  prohibition_areas_layer
  prohibition_areas_tool
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN "*.svn" EXCLUDE
)

# 安装插件描述文件
install(FILES
  costmap_plugins.xml
  rviz_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 安装配置和启动文件
install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*.svn" EXCLUDE
)

# 安装图标文件
install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.svg"
)

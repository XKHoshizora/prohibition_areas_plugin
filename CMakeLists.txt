cmake_minimum_required(VERSION 3.0.2)
project(map_restriction_plugin)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rviz
    costmap_2d
    geometry_msgs
    nav_msgs
    pluginlib
)

find_package(yaml-cpp REQUIRED)

# 添加消息生成
add_message_files(
    FILES
    Restriction.msg
    RestrictionArray.msg
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp rviz costmap_2d geometry_msgs nav_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

# RViz插件库
add_library(${PROJECT_NAME}_rviz_plugins
    src/draw_tool.cpp
)
target_link_libraries(${PROJECT_NAME}_rviz_plugins
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)

# Costmap图层插件库
add_library(${PROJECT_NAME}_layer
    src/restriction_layer.cpp
    src/restriction_data.cpp
)
target_link_libraries(${PROJECT_NAME}_layer
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)

# 节点
add_executable(save_node src/save_node.cpp)
target_link_libraries(save_node
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)

add_executable(load_node src/load_node.cpp)
target_link_libraries(load_node
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)

# 插件描述文件
install(FILES rviz_plugin/plugins.xml restriction_layer_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 安装目标
install(TARGETS ${PROJECT_NAME}_rviz_plugins ${PROJECT_NAME}_layer save_node load_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
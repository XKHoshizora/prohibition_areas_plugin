// src/restriction_layer.cpp
#include "map_restriction_plugin/restriction_layer.h"

#include <pluginlib/class_list_macros.h>

namespace map_restriction_plugin {

RestrictionLayer::RestrictionLayer()
    : map_min_x_(0), map_min_y_(0), map_max_x_(0), map_max_y_(0) {}

RestrictionLayer::~RestrictionLayer() {}

void RestrictionLayer::onInitialize() {
    ros::NodeHandle nh("~/" + name_);
    current_ = true;

    // 从参数服务器加载配置文件路径
    nh.param("config_file", config_file_, std::string(""));
    if (!config_file_.empty()) {
        if (!parseRestrictionsFromYaml()) {
            ROS_ERROR("Failed to load restrictions from %s",
                      config_file_.c_str());
        } else {
            ROS_INFO("Loaded restrictions from %s", config_file_.c_str());
        }
    }

    // 订阅禁区话题
    restriction_sub_ = nh.subscribe(
        "/map_restrictions", 1, &RestrictionLayer::restrictionCallback, this);

    ROS_INFO("Restriction layer initialized");
}

// 更新代价地图边界
void RestrictionLayer::updateBounds(double robot_x, double robot_y,
                                    double robot_yaw, double* min_x,
                                    double* min_y, double* max_x,
                                    double* max_y) {
    if (!enabled_) return;

    boost::recursive_mutex::scoped_lock lock(mutex_);

    // 使用预计算的地图边界更新代价地图边界
    *min_x = std::min(*min_x, map_min_x_);
    *min_y = std::min(*min_y, map_min_y_);
    *max_x = std::max(*max_x, map_max_x_);
    *max_y = std::max(*max_y, map_max_y_);
}

// 更新代价值
void RestrictionLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                                   int min_i, int min_j, int max_i, int max_j) {
    if (!enabled_) return;

    boost::recursive_mutex::scoped_lock lock(mutex_);

    // 遍历指定范围内的所有单元格
    for (int j = min_j; j < max_j; j++) {
        for (int i = min_i; i < max_i; i++) {
            // 获取单元格的世界坐标
            double wx, wy;
            master_grid.mapToWorld(i, j, wx, wy);

            geometry_msgs::Point point;
            point.x = wx;
            point.y = wy;

            // 如果点在禁区内，设置为致命障碍物
            if (restrictions_.isPointRestricted(point)) {
                master_grid.setCost(i, j, costmap_2d::LETHAL_OBSTACLE);
            }
        }
    }
}

// 处理接收到的禁区消息
void RestrictionLayer::restrictionCallback(const RestrictionArray& msg) {
    boost::recursive_mutex::scoped_lock lock(mutex_);

    // 清除现有禁区
    restrictions_ = RestrictionData();

    // 添加新的禁区
    for (const auto& restriction : msg.restrictions) {
        restrictions_.addRestriction(restriction);
    }

    // 重新计算地图边界
    computeMapBounds();

    ROS_INFO("Updated restrictions from message");
}

}  // namespace map_restriction_plugin

PLUGINLIB_EXPORT_CLASS(map_restriction_plugin::RestrictionLayer,
                       costmap_2d::Layer)
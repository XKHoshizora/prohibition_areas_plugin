// src/restriction_data.cpp
#include "map_restriction_plugin/restriction_data.h"

#include <yaml-cpp/yaml.h>

namespace map_restriction_plugin {

void RestrictionData::addRestriction(const Restriction& restriction) {
    restrictions_.push_back(restriction);
}

void RestrictionData::removeRestriction(size_t index) {
    if (index < restrictions_.size()) {
        restrictions_.erase(restrictions_.begin() + index);
    }
}

void RestrictionData::updateRestriction(size_t index,
                                        const Restriction& restriction) {
    if (index < restrictions_.size()) {
        restrictions_[index] = restriction;
    }
}

// 判断点是否在禁区内
bool RestrictionData::isPointRestricted(
    const geometry_msgs::Point& point) const {
    for (const auto& restriction : restrictions_) {
        // 根据禁区类型判断点是否在内
        switch (restriction.type) {
            case RECTANGLE: {
                // 矩形判定：检查点是否在矩形范围内
                if (restriction.points.size() != 4) continue;
                double minX = std::min(
                    {restriction.points[0].x, restriction.points[1].x,
                     restriction.points[2].x, restriction.points[3].x});
                double maxX = std::max(
                    {restriction.points[0].x, restriction.points[1].x,
                     restriction.points[2].x, restriction.points[3].x});
                double minY = std::min(
                    {restriction.points[0].y, restriction.points[1].y,
                     restriction.points[2].y, restriction.points[3].y});
                double maxY = std::max(
                    {restriction.points[0].y, restriction.points[1].y,
                     restriction.points[2].y, restriction.points[3].y});
                if (point.x >= minX && point.x <= maxX && point.y >= minY &&
                    point.y <= maxY) {
                    return true;
                }
                break;
            }
            case CIRCLE: {
                // 圆形判定：检查点到圆心距离是否小于半径
                if (restriction.points.empty()) continue;
                double dx = point.x - restriction.points[0].x;
                double dy = point.y - restriction.points[0].y;
                if (dx * dx + dy * dy <=
                    restriction.radius * restriction.radius) {
                    return true;
                }
                break;
            }
            case POLYGON: {
                // 多边形判定：使用射线法判断点是否在多边形内
                if (restriction.points.size() < 3) continue;
                bool inside = false;
                size_t j = restriction.points.size() - 1;
                for (size_t i = 0; i < restriction.points.size(); i++) {
                    if ((restriction.points[i].y > point.y) !=
                            (restriction.points[j].y > point.y) &&
                        point.x < (restriction.points[j].x -
                                   restriction.points[i].x) *
                                          (point.y - restriction.points[i].y) /
                                          (restriction.points[j].y -
                                           restriction.points[i].y) +
                                      restriction.points[i].x) {
                        inside = !inside;
                    }
                    j = i;
                }
                if (inside) return true;
                break;
            }
        }
    }
    return false;
}

// 保存禁区数据到YAML文件
bool RestrictionData::saveToFile(const std::string& filename) const {
    try {
        YAML::Node root;
        for (const auto& restriction : restrictions_) {
            YAML::Node node;
            // 保存禁区类型
            node["type"] = static_cast<int>(restriction.type);

            // 保存点数据
            YAML::Node points;
            for (const auto& point : restriction.points) {
                YAML::Node p;
                p["x"] = point.x;
                p["y"] = point.y;
                points.push_back(p);
            }
            node["points"] = points;

            // 如果是圆形，保存半径
            if (restriction.type == CIRCLE) {
                node["radius"] = restriction.radius;
            }
            root["restrictions"].push_back(node);
        }

        std::ofstream fout(filename);
        if (!fout) return false;
        fout << root;
        return true;
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Failed to save restrictions: %s", e.what());
        return false;
    }
}

// 从YAML文件加载禁区数据
bool RestrictionData::loadFromFile(const std::string& filename) {
    try {
        YAML::Node root = YAML::LoadFile(filename);
        if (!root["restrictions"]) return false;

        restrictions_.clear();
        for (const auto& node : root["restrictions"]) {
            Restriction restriction;
            restriction.type = static_cast<uint8_t>(node["type"].as<int>());

            // 加载点数据
            for (const auto& p : node["points"]) {
                geometry_msgs::Point point;
                point.x = p["x"].as<double>();
                point.y = p["y"].as<double>();
                point.z = 0;
                restriction.points.push_back(point);
            }

            // 如果是圆形，加载半径
            if (restriction.type == CIRCLE) {
                restriction.radius = node["radius"].as<double>();
            }

            restrictions_.push_back(restriction);
        }
        return true;
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Failed to load restrictions: %s", e.what());
        return false;
    }
}

}  // namespace map_restriction_plugin
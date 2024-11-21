// src/draw_tool.cpp
#include "map_restriction_plugin/draw_tool.h"

#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/MarkerArray.h>

namespace map_restriction_plugin {

DrawTool::DrawTool() : current_type_(POLYGON), drawing_(false) {
    shortcut_key_ = 'r';  // 设置快捷键为'r'
}

DrawTool::~DrawTool() {}

// 工具初始化
void DrawTool::onInitialize() {
    // 创建ROS发布器，用于发布禁区数据和可视化标记
    restriction_pub_ = nh_.advertise<RestrictionArray>("/map_restrictions", 1);

    // 添加类型选择属性到RViz界面
    restriction_type_property_ =
        new rviz::EnumProperty("Restriction Type", "Polygon",
                               "选择要绘制的禁区类型", getPropertyContainer());

    // 添加禁区类型选项
    restriction_type_property_->addOption("Rectangle", RECTANGLE);
    restriction_type_property_->addOption("Polygon", POLYGON);
    restriction_type_property_->addOption("Circle", CIRCLE);

    ROS_INFO("Map restriction draw tool initialized");
}

void DrawTool::activate() {
    drawing_ = false;
    current_points_.clear();
    updateVisualization();
}

void DrawTool::deactivate() {
    drawing_ = false;
    current_points_.clear();
    updateVisualization();
}

// 处理鼠标事件
int DrawTool::processMouseEvent(rviz::ViewportMouseEvent& event) {
    // 获取当前选择的禁区类型
    current_type_ = restriction_type_property_->getOptionInt();

    if (event.leftDown()) {
        // 鼠标左键按下，开始或继续绘制
        if (!drawing_) {
            drawing_ = true;
            current_points_.clear();
        }

        // 获取鼠标点击位置
        Ogre::Vector3 intersection;
        Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
        if (getPointOnPlaneFromMouse(event.viewport, ground_plane,
                                     intersection)) {
            geometry_msgs::Point point;
            point.x = intersection.x;
            point.y = intersection.y;
            point.z = 0.0;

            current_points_.push_back(point);
            updateVisualization();

            // 如果是圆形且已有两个点，自动完成绘制
            if (current_type_ == CIRCLE && current_points_.size() == 2) {
                finishDrawing();
            }
        }
    } else if (event.rightDown()) {
        // 鼠标右键完成绘制
        if (drawing_) {
            finishDrawing();
        }
    }

    return Render;
}

// 完成绘制并发布禁区数据
void DrawTool::finishDrawing() {
    if (current_points_.size() < 3 && current_type_ == POLYGON) {
        ROS_WARN("Not enough points for polygon, need at least 3");
        return;
    }

    Restriction restriction;
    restriction.type = current_type_;
    restriction.points = current_points_;

    if (current_type_ == CIRCLE) {
        // 计算圆形半径
        double dx = current_points_[1].x - current_points_[0].x;
        double dy = current_points_[1].y - current_points_[0].y;
        restriction.radius = sqrt(dx * dx + dy * dy);
    }

    // 发布禁区数据
    RestrictionArray msg;
    msg.header.frame_id = context_->getFixedFrame().toStdString();
    msg.header.stamp = ros::Time::now();
    msg.restrictions.push_back(restriction);
    restriction_pub_.publish(msg);

    drawing_ = false;
    current_points_.clear();
    updateVisualization();
}

// 更新可视化显示
void DrawTool::updateVisualization() {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    marker.header.frame_id = context_->getFixedFrame().toStdString();
    marker.header.stamp = ros::Time::now();
    marker.ns = "drawing_markers";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;  // 线宽
    marker.color.r = 1.0;  // 红色
    marker.color.a = 1.0;  // 不透明度

    if (drawing_ && !current_points_.empty()) {
        if (current_type_ == CIRCLE && current_points_.size() == 2) {
            // 绘制圆形
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.pose.position = current_points_[0];
            double dx = current_points_[1].x - current_points_[0].x;
            double dy = current_points_[1].y - current_points_[0].y;
            double radius = sqrt(dx * dx + dy * dy);
            marker.scale.x = marker.scale.y = radius * 2;
            marker.scale.z = 0.1;
        } else {
            // 绘制线条
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.points = current_points_;
            if (current_type_ != CIRCLE && current_points_.size() > 1) {
                marker.points.push_back(current_points_[0]);  // 闭合多边形
            }
        }
        markers.markers.push_back(marker);
    }

    // 发布标记进行可视化
    marker_pub_.publish(markers);
}

}  // namespace map_restriction_plugin

PLUGINLIB_EXPORT_CLASS(map_restriction_plugin::DrawTool, rviz::Tool)
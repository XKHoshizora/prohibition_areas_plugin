// prohibition_areas_display.cpp
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <pluginlib/class_list_macros.h>
#include <prohibition_areas_plugin/prohibition_areas_tool/prohibition_areas_display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ros/package.h>

namespace prohibition_areas_tool {

ProhibitionAreasDisplay::ProhibitionAreasDisplay() {
    color_property_ = new rviz::ColorProperty("Color", QColor(0, 255, 0),
                                              "Color to draw the areas.", this,
                                              SLOT(updateVisualProperties()));

    selected_color_property_ =
        new rviz::ColorProperty("Selected Color", QColor(255, 165, 0),
                                "Color to draw the selected area.", this,
                                SLOT(updateVisualProperties()));

    line_width_property_ =
        new rviz::FloatProperty("Line Width", 0.1, "Width of the lines.", this,
                                SLOT(updateVisualProperties()));

    point_size_property_ =
        new rviz::FloatProperty("Point Size", 0.2, "Size of the points.", this,
                                SLOT(updateVisualProperties()));

    fill_property_ = new rviz::BoolProperty(
        "Fill Areas", true, "Fill the areas with semi-transparent color.", this,
        SLOT(updateVisualProperties()));
}

ProhibitionAreasDisplay::~ProhibitionAreasDisplay() { clear(); }

void ProhibitionAreasDisplay::onInitialize() {
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
}

void ProhibitionAreasDisplay::onEnable() { scene_node_->setVisible(true); }

void ProhibitionAreasDisplay::onDisable() { scene_node_->setVisible(false); }

void ProhibitionAreasDisplay::clear() {
    for (auto& obj : area_objects_) {
        scene_manager_->destroyManualObject(obj.second);
    }
    area_objects_.clear();
}

void ProhibitionAreasDisplay::update(float wall_dt, float ros_dt) {
    updateAreas();
}

void ProhibitionAreasDisplay::updateVisualProperties() { createGeometry(); }

void ProhibitionAreasDisplay::updateAreas() { createGeometry(); }

void ProhibitionAreasDisplay::createGeometry() {
    clear();

    // 获取禁区文件路径
    std::string pkg_path = ros::package::getPath("prohibition_areas_plugin");
    std::string file_path = pkg_path + "/prohibition_areas/prohibition_areas.yaml";

    try {
        YAML::Node root = YAML::LoadFile(file_path);
        if (!root.IsSequence()) {
            ROS_ERROR("Invalid prohibition areas file format");
            return;
        }

        // 获取显示属性
        QColor color = color_property_->getColor();
        QColor selected_color = selected_color_property_->getColor();
        float line_width = line_width_property_->getFloat();
        float point_size = point_size_property_->getFloat();
        bool fill = fill_property_->getBool();

        // 遍历所有禁区
        for (const auto& area_node : root) {
            if (!area_node["points"] || !area_node["name"]) continue;

            std::string area_name = area_node["name"].as<std::string>();

            // 创建显示对象
            Ogre::ManualObject* manual = scene_manager_->createManualObject();
            manual->begin("BaseWhiteNoLighting",
                fill ? Ogre::RenderOperation::OT_TRIANGLE_FAN
                    : Ogre::RenderOperation::OT_LINE_STRIP);

            // 设置材质属性
            manual->colour(color.redF(), color.greenF(), color.blueF(),
                fill ? 0.5f : 1.0f);

            // 添加顶点
            const auto& points = area_node["points"];
            for (const auto& point : points) {
                if (!point.IsSequence() || point.size() < 2) continue;
                manual->position(point[0].as<float>(), point[1].as<float>(), 0.0f);
            }

            // 如果是封闭多边形，添加首个顶点以闭合
            if (points.size() >= 3) {
                manual->position(points[0][0].as<float>(), points[0][1].as<float>(), 0.0f);
            }

            manual->end();

            // 设置线宽
            if (!fill) {
                manual->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
                manual->setLineWidth(line_width);
            }

            scene_node_->attachObject(manual);
            area_objects_[area_name] = manual;
        }
    } catch (const YAML::Exception& e) {
        ROS_ERROR_STREAM("Failed to load prohibition areas: " << e.what());
    }
}

}  // end namespace prohibition_areas_tool

PLUGINLIB_EXPORT_CLASS(prohibition_areas_tool::ProhibitionAreasDisplay,
                       rviz::Display)
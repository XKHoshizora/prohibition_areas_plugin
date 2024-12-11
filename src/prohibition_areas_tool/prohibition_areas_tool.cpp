// prohibition_areas_tool.cpp
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgrePlane.h>
#include <prohibition_areas_plugin/prohibition_areas_tool/edit_points_frame.h>
#include <prohibition_areas_plugin/prohibition_areas_tool/prohibition_areas_tool.h>
#include <prohibition_areas_plugin/prohibition_areas_tool/prohibition_areas_saver.h>
#include <rviz/display_context.h>
#include <rviz/geometry.h>
#include <rviz/properties/string_property.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <ros/package.h>
#include <QApplication>
#include <QMessageBox>
#include <QDateTime>

namespace prohibition_areas_tool {

ProhibitionAreasTool::ProhibitionAreasTool()
    : Tool(),
      preview_node_(nullptr),
      preview_object_(nullptr),
      edit_frame_(nullptr),
      drawing_(false) {
    shortcut_key_ = 'p';

    frame_property_ = new rviz::StringProperty(
        "Frame ID", "map", "The frame ID for the prohibition areas.",
        getPropertyContainer());
}

ProhibitionAreasTool::~ProhibitionAreasTool() {
    delete edit_frame_;

    if (preview_object_) {
        scene_manager_->destroyManualObject(preview_object_);
    }
    if (preview_node_) {
        scene_manager_->destroySceneNode(preview_node_);
    }
}

void ProhibitionAreasTool::onInitialize() {
    // 创建编辑面板
    edit_frame_ = new EditPointsFrame();
    connect(edit_frame_, &EditPointsFrame::pointsModified, this,
            &ProhibitionAreasTool::areaUpdated);

    // 创建预览显示
    preview_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    preview_object_ = scene_manager_->createManualObject();
    preview_object_->setDynamic(true);
    preview_node_->attachObject(preview_object_);

    // 创建材质
    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
        "ProhibitionAreaMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material->setPointSize(10);
    material->setAmbient(1.0, 1.0, 1.0);
    material->setDiffuse(1.0, 1.0, 1.0, 1.0);

    // 设置工具光标
    setCursor(QCursor(Qt::CrossCursor));
}

void ProhibitionAreasTool::activate() {
    if (edit_frame_) {
        edit_frame_->show();
    }
    drawing_ = true;
    current_points_.clear();
    updatePreview();
    preview_node_->setVisible(true);
}

void ProhibitionAreasTool::deactivate() {
    if (!current_points_.empty()) {
        QMessageBox::StandardButton reply = QMessageBox::question(
            nullptr, "Confirm", "You have unsaved changes. Do you want to discard them?",
            QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::No) {
            return;
        }
    }

    if (edit_frame_) {
        edit_frame_->hide();
    }
    drawing_ = false;
    current_points_.clear();
    updatePreview();

    if (preview_object_) {
        preview_object_->clear();
    }
    if (preview_node_) {
        preview_node_->setVisible(false);
    }
}

int ProhibitionAreasTool::processMouseEvent(rviz::ViewportMouseEvent& event) {
    if (!drawing_) return Render;

    geometry_msgs::Point point;
    bool valid_point = getWorldPoint(event, point);

    // 左键点击添加点
    if (valid_point && event.leftDown()) {
        addPoint(point);
        updatePreview();  // 立即更新显示
        return Render;
    }

    // 右键点击完成绘制
    if (event.rightDown()) {
        if (current_points_.size() >= 3) {
            saveCurrentArea();
            current_points_.clear();
            updatePreview();
            Q_EMIT areaUpdated();
        } else {
            QMessageBox::warning(nullptr, "Warning",
                "At least 3 points are required to create an area.");
        }
        return Render;
    }

    // if (!getWorldPoint(event, point)) return Render;

    // if (event.leftDown()) {
    //     addPoint(point);
    // } else if (event.rightDown()) {
    //     saveCurrentArea();
    //     current_points_.clear();
    //     Q_EMIT areaUpdated();
    // }

    // updatePreview();

    // 鼠标移动时预览下一个点
    if (valid_point && !current_points_.empty()) {
        preview_object_->clear();

        // 绘制已有的线段
        preview_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
        preview_object_->colour(0.0f, 1.0f, 0.0f, 1.0f);

        for (const auto& p : current_points_) {
            preview_object_->position(p.x, p.y, 0.0f);
        }

        // 添加预览线段到鼠标位置
        preview_object_->position(point.x, point.y, 0.0f);

        if (current_points_.size() >= 2) {
            // 添加到起始点的虚线
            preview_object_->colour(0.0f, 1.0f, 0.0f, 0.5f);
            preview_object_->position(current_points_[0].x, current_points_[0].y, 0.0f);
        }

        preview_object_->end();

        // 绘制点
        preview_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);
        preview_object_->colour(1.0f, 0.0f, 0.0f, 1.0f);

        for (const auto& p : current_points_) {
            preview_object_->position(p.x, p.y, 0.0f);
        }

        preview_object_->end();
    }

    return Render;
}

bool ProhibitionAreasTool::getWorldPoint(rviz::ViewportMouseEvent& event,
                                         geometry_msgs::Point& point) {
    Ogre::Vector3 pos;

    // 创建一个平面用于投影
    Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);

    if (!rviz::getPointOnPlaneFromWindowXY(
            event.viewport, ground_plane, event.x,
            event.y, pos)) {
        return false;
    }

    point.x = pos.x;
    point.y = pos.y;
    point.z = 0.0f;
    return true;
}

void ProhibitionAreasTool::updatePreview() {
    // 清除之前的显示
    preview_object_->clear();

    if (current_points_.empty()) {
        preview_node_->setVisible(false);
        return;
    }

    preview_node_->setVisible(true);
    preview_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);

    // 开始绘制点和线
    preview_object_->begin("ProhibitionAreaMaterial", Ogre::RenderOperation::OT_LINE_STRIP);

    // 设置线的颜色（绿色）
    preview_object_->colour(0.0f, 1.0f, 0.0f, 1.0f);

    // 添加所有已有的点
    for (const auto& point : current_points_) {
        preview_object_->position(point.x, point.y, 0.0f);
    }

    // 如果有至少3个点，自动闭合多边形
    if (current_points_.size() >= 3) {
        preview_object_->position(current_points_[0].x, current_points_[0].y, 0.0f);
    }

    preview_object_->end();

    // 绘制点标记
    preview_object_->begin("ProhibitionAreaMaterial", Ogre::RenderOperation::OT_POINT_LIST);
    preview_object_->colour(1.0f, 0.0f, 0.0f, 1.0f); // 红色点

    for (const auto& point : current_points_) {
        // 绘制每个点
        preview_object_->position(point.x, point.y, 0.0f);
    }

    preview_object_->end();
}

void ProhibitionAreasTool::saveCurrentArea() {
    if (current_points_.size() < 3) {
        ROS_ERROR("Cannot save area: at least 3 points required");
        QMessageBox::warning(nullptr, "Error",
            "Cannot save area: at least 3 points required");
        return;
    }

    if (current_area_id_.empty()) {
        // 生成唯一的区域ID
        current_area_id_ = "area_" +
            QString::number(QDateTime::currentMSecsSinceEpoch()).toStdString();
        ROS_INFO_STREAM("Generated new area ID: " << current_area_id_);
    }

    // 更新编辑面板
    edit_frame_->setAreaPoints(current_area_id_, current_points_);

    // 立即触发显示更新
    Q_EMIT areaUpdated();

    // 自动保存到文件
    ProhibitionArea area;
    area.name = current_area_id_;
    area.frame_id = frame_property_->getStdString();
    area.points = current_points_;

    std::vector<ProhibitionArea> areas{area};
    std::string pkg_path = ros::package::getPath("prohibition_areas_plugin");
    std::string file_path = pkg_path + "/prohibition_areas/prohibition_areas.yaml";

    if (ProhibitionAreasSaver::saveToFile(areas, file_path)) {
        ROS_INFO("Area saved successfully");
    } else {
        ROS_ERROR("Failed to save area");
        QMessageBox::critical(nullptr, tr("Error"), tr("Failed to save area to file"));
    }
}

void ProhibitionAreasTool::addPoint(const geometry_msgs::Point& point) {
    current_points_.push_back(point);
    if (edit_frame_) {
        edit_frame_->setAreaPoints(current_area_id_, current_points_);
    }
}

}  // end namespace prohibition_areas_tool

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(prohibition_areas_tool::ProhibitionAreasTool, rviz::Tool)
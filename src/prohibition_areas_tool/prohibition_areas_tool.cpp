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
#include <QInputDialog>
#include <sys/stat.h>

namespace prohibition_areas_tool {

struct ProhibitionAreasTool::ROSPublishers {
    ros::Publisher preview_pub;
};

ProhibitionAreasTool::ProhibitionAreasTool()
    : Tool(),
      preview_node_(nullptr),
      preview_object_(nullptr),
      edit_frame_(nullptr),
      drawing_(false),
      ros_publishers_(new ROSPublishers) {
    shortcut_key_ = 'p';

    frame_property_ = new rviz::StringProperty(
        "Frame ID", "map", "The frame ID for the prohibition areas.",
        getPropertyContainer());
}

ProhibitionAreasTool::~ProhibitionAreasTool() = default;

void ProhibitionAreasTool::onInitialize() {
    // 获取私有节点句柄
    ros::NodeHandle private_nh("~");

    // 读取保存路径，如果没有配置则使用默认路径
    std::string pkg_path = ros::package::getPath("prohibition_areas_plugin");
    save_path_ = pkg_path + "/prohibition_areas/prohibition_areas.yaml";
    private_nh.param<std::string>("prohibition_areas_path", save_path_, save_path_);

    // 读取是否为追加模式，默认为true
    private_nh.param<bool>("append_mode", append_mode_, true);

    edit_frame_ = new EditPointsFrame(save_path_);
    connect(edit_frame_, &EditPointsFrame::pointsModified, this, [this]() {
        areaUpdated();
        // 触发代价地图重新加载
        if (context_) {
            // 发布一个标记消息，通知代价地图层重新加载
            ros::NodeHandle nh;
            ros::Publisher update_pub =
                nh.advertise<std_msgs::Empty>("prohibition_areas_update", 1);
            std_msgs::Empty msg;
            update_pub.publish(msg);
        }
    });

    // 创建编辑面板
    edit_frame_ = new EditPointsFrame(save_path_);  // 传递保存路径给编辑面板
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

    // 初始化预览发布器
    ros::NodeHandle nh;
    ros_publishers_->preview_pub = nh.advertise<prohibition_areas_plugin::ProhibitionAreas>(
        "prohibition_areas_preview", 1);

    // 连接编辑框架的信号
    connect(edit_frame_, &EditPointsFrame::pointsModified, this, [this]() {
        publishPreview();  // 发布预览
        areaUpdated();
    });
}

void ProhibitionAreasTool::publishPreview() {
    prohibition_areas_plugin::ProhibitionAreas msg;

    // 获取所有区域数据
    std::vector<ProhibitionArea> areas = edit_frame_->getAllAreas();

    // 转换为消息格式
    for (const auto& area : areas) {
        prohibition_areas_plugin::ProhibitionArea area_msg;
        area_msg.name = area.name;
        area_msg.frame_id = frame_property_->getStdString();
        area_msg.points = area.points;
        msg.areas.push_back(area_msg);
    }

    // 添加当前正在编辑的区域
    if (!current_points_.empty()) {
        prohibition_areas_plugin::ProhibitionArea current_area;
        current_area.name = "editing";
        current_area.frame_id = frame_property_->getStdString();
        current_area.points.insert(current_area.points.end(),
                                 current_points_.begin(),
                                 current_points_.end());
        msg.areas.push_back(current_area);
    }

    ros_publishers_->preview_pub.publish(msg);
}

void ProhibitionAreasTool::activate() {
    if (edit_frame_) {
        edit_frame_->show();
    }
    // 清除当前点和区域ID，准备创建新的禁区
    current_area_id_.clear();
    current_points_.clear();
    drawing_ = true;
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
            // 获取新区域名称
            bool ok;
            QString area_name = QInputDialog::getText(nullptr,
                "Save Area",
                "Enter area name:",
                QLineEdit::Normal,
                QString::fromStdString(current_area_id_),
                &ok);

            if (ok && !area_name.isEmpty()) {
                // 保存当前区域
                std::string new_area_id = area_name.toStdString();
                if (edit_frame_) {
                    edit_frame_->setAreaPoints(new_area_id, current_points_);
                }

                // 清除当前点，准备下一个区域
                current_area_id_.clear();
                current_points_.clear();
                updatePreview();
                Q_EMIT areaUpdated();
            }
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

bool ProhibitionAreasTool::saveAreas(const std::vector<ProhibitionArea>& new_areas) {
    std::vector<ProhibitionArea> areas;

    struct stat buffer;
    bool file_exists = (stat(save_path_.c_str(), &buffer) == 0);

    // 如果是追加模式，先读取现有的禁区
    if (append_mode_ && file_exists) {
        if (!ProhibitionAreasSaver::loadFromFile(areas, save_path_)) {
            ROS_WARN("Failed to load existing areas, starting fresh");
        }
    }

    // 更新或添加新区域
    for (const auto& new_area : new_areas) {
        bool area_updated = false;
        for (auto& area : areas) {
            if (area.name == new_area.name) {
                area = new_area;
                area_updated = true;
                break;
            }
        }
        if (!area_updated) {
            areas.push_back(new_area);
        }
    }

    // 保存所有区域
    bool success = ProhibitionAreasSaver::saveToFile(areas, save_path_);
    if (success) {
        ROS_INFO_STREAM("Successfully saved prohibition areas to: " << save_path_);
    } else {
        ROS_ERROR_STREAM("Failed to save prohibition areas to: " << save_path_);
    }
    return success;
}

bool ProhibitionAreasTool::saveCurrentArea() {
    if (current_points_.size() < 3) {
        ROS_ERROR("Cannot save area: at least 3 points required");
        QMessageBox::warning(nullptr, "Error",
            "Cannot save area: at least 3 points required");
        return false;
    }

    // 获取区域名称
    bool ok;
    QString area_name = QInputDialog::getText(nullptr,
        "Save Area",
        "Enter area name:",
        QLineEdit::Normal,
        QString::fromStdString(current_area_id_),
        &ok);

    if (!ok || area_name.isEmpty()) {
        QMessageBox::warning(nullptr, "Warning", "Area name is required.");
        return false;
    }

    // 创建新区域
    ProhibitionArea area;
    area.name = area_name.toStdString();
    area.frame_id = frame_property_->getStdString();
    area.points = current_points_;

    // 保存区域
    std::vector<ProhibitionArea> areas{area};
    if (saveAreas(areas)) {
        // 更新编辑面板
        if (edit_frame_) {
            edit_frame_->setAreaPoints(area.name, current_points_);
            // 不需要显式调用 pointsModified，因为 setAreaPoints 会触发它
        }
        return true;
    }
    return false;
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
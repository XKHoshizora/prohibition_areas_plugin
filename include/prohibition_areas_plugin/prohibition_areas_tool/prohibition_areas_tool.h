// prohibition_areas_tool.h
#ifndef PROHIBITION_AREAS_TOOL_H
#define PROHIBITION_AREAS_TOOL_H

#include <geometry_msgs/Point.h>
#include <rviz/tool.h>

#include <QCursor>
#include <QObject>
#include <memory>
#include <string>
#include <vector>

#include "edit_points_frame.h"
#include "prohibition_areas_saver.h"
#include <prohibition_areas_plugin/ProhibitionArea.h>    // 添加单个禁区消息
#include <prohibition_areas_plugin/ProhibitionAreas.h>   // 添加禁区列表消息

namespace Ogre {
class SceneNode;
class ManualObject;
}  // namespace Ogre

namespace rviz {
class ViewportMouseEvent;
class StringProperty;
}  // namespace rviz

namespace ros {
    class Publisher;
    class NodeHandle;
} // namespace ros

namespace prohibition_areas_tool {

class EditPointsFrame;

class ProhibitionAreasTool : public rviz::Tool {
    Q_OBJECT
   public:
    ProhibitionAreasTool();
    ~ProhibitionAreasTool() override;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;
    int processMouseEvent(rviz::ViewportMouseEvent& event) override;

   Q_SIGNALS:
    void areaUpdated();

   protected:
    // 转换屏幕坐标到世界坐标
    bool getWorldPoint(rviz::ViewportMouseEvent& event,
                       geometry_msgs::Point& point);
    // 更新预览显示
    void updatePreview();
    // 保存当前区域
    bool saveCurrentArea();
    // 添加点到当前区域
    void addPoint(const geometry_msgs::Point& point);
    // 保存区域
    bool saveAreas(const std::vector<ProhibitionArea>& areas);
    // 发布预览数据
    void publishPreview();

   private:
    // 当前编辑的区域ID
    std::string current_area_id_;
    // 当前区域的点集
    std::vector<geometry_msgs::Point> current_points_;
    // 预览显示节点
    Ogre::SceneNode* preview_node_;
    // 预览显示对象
    Ogre::ManualObject* preview_object_;
    // 编辑控制面板
    EditPointsFrame* edit_frame_;
    // 状态标志
    bool drawing_;
    // 框架ID属性
    rviz::StringProperty* frame_property_;
    // 保存路径
    std::string save_path_;
    // 保存模式
    bool append_mode_;

    // 使用智能指针管理 ROS publisher
    struct ROSPublishers;
    std::unique_ptr<ROSPublishers> ros_publishers_;
};

}  // end namespace prohibition_areas_tool

#endif  // PROHIBITION_AREAS_TOOL_H
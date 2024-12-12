// prohibition_areas_display.h
#ifndef PROHIBITION_AREAS_DISPLAY_H
#define PROHIBITION_AREAS_DISPLAY_H

#include <geometry_msgs/PolygonStamped.h>
#include <rviz/display.h>
#include <prohibition_areas_plugin/ProhibitionArea.h>    // 添加单个禁区消息
#include <prohibition_areas_plugin/ProhibitionAreas.h>   // 添加禁区列表消息

#include <map>
#include <string>

namespace Ogre {
class SceneNode;
class ManualObject;
}  // namespace Ogre

namespace rviz {
class ColorProperty;
class FloatProperty;
class BoolProperty;
}  // namespace rviz

namespace prohibition_areas_tool {

class ProhibitionAreasDisplay : public rviz::Display {
    Q_OBJECT
   public:
    ProhibitionAreasDisplay();
    ~ProhibitionAreasDisplay() override;

   protected:
    void onInitialize() override;
    void onEnable() override;
    void onDisable() override;
    void update(float wall_dt, float ros_dt) override;

   private Q_SLOTS:
    void updateVisualProperties();
    void updateAreas();

   private:
    void clear();
    void createGeometry();  // 更新显示函数
    void previewCallback(const prohibition_areas_plugin::ProhibitionAreas::ConstPtr& msg);

    // 预览订阅器
    ros::Subscriber preview_sub_;

    // 场景节点
    Ogre::SceneNode* scene_node_;
    // 区域显示对象
    std::map<std::string, Ogre::ManualObject*> area_objects_;

    // 显示属性
    rviz::ColorProperty* color_property_;
    rviz::ColorProperty* selected_color_property_;
    rviz::FloatProperty* line_width_property_;
    rviz::FloatProperty* point_size_property_;
    rviz::BoolProperty* fill_property_;
};

}  // end namespace prohibition_areas_tool

#endif  // PROHIBITION_AREAS_DISPLAY_H
// include/map_restriction_plugin/draw_tool.h
#pragma once
#include <rviz/tool.h>

#include "restriction_data.h"

namespace map_restriction_plugin {

// RViz绘制工具类
class DrawTool : public rviz::Tool {
   public:
    DrawTool();
    virtual ~DrawTool();

    // RViz工具初始化
    virtual void onInitialize();
    // 工具激活
    virtual void activate();
    // 工具停用
    virtual void deactivate();
    // 处理鼠标事件
    virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

   private:
    uint8_t current_type_;                              // 当前绘制类型
    std::vector<geometry_msgs::Point> current_points_;  // 当前绘制点
    bool drawing_;                                      // 是否正在绘制
    ros::Publisher restriction_pub_;                    // 禁区发布器
    ros::NodeHandle nh_;                                // ROS节点句柄
    void updateVisualization();                         // 更新可视化
};

}  // namespace map_restriction_plugin
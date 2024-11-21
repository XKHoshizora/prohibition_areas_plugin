// include/map_restriction_plugin/restriction_layer.h
#pragma once
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

#include "restriction_data.h"

namespace map_restriction_plugin {

// 代价地图禁区图层类
class RestrictionLayer : public costmap_2d::Layer {
   public:
    RestrictionLayer();
    virtual ~RestrictionLayer();

    // 图层初始化
    virtual void onInitialize();
    // 更新边界
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                              double* min_x, double* min_y, double* max_x,
                              double* max_y);
    // 更新代价值
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i,
                             int min_j, int max_i, int max_j);

   private:
    void restrictionCallback(const RestrictionArray& msg);  // 禁区消息回调
    bool parseRestrictionsFromYaml();                       // 解析YAML配置
    void computeMapBounds();  // 计算地图边界

    RestrictionData restrictions_;     // 禁区数据
    ros::Subscriber restriction_sub_;  // 禁区订阅器
    std::string config_file_;          // 配置文件路径
    double map_min_x_, map_min_y_, map_max_x_, map_max_y_;  // 地图边界
    mutable boost::recursive_mutex mutex_;                  // 互斥锁
};

}  // namespace map_restriction_plugin
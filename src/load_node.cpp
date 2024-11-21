// src/load_node.cpp
#include <ros/ros.h>

#include "map_restriction_plugin/RestrictionArray.h"
#include "map_restriction_plugin/restriction_data.h"

class LoadNode {
   public:
    LoadNode() : nh_("~") {
        // 获取加载路径参数
        if (!nh_.getParam("load_path", load_path_)) {
            load_path_ = "restrictions.yaml";
            ROS_WARN("No load_path specified, using default: %s",
                     load_path_.c_str());
        }

        // 创建发布器
        restriction_pub_ =
            nh_.advertise<map_restriction_plugin::RestrictionArray>(
                "/map_restrictions", 1, true);

        // 设置定时器，定期尝试加载文件
        load_timer_ = nh_.createTimer(ros::Duration(1.0),
                                      &LoadNode::loadTimerCallback, this);

        ROS_INFO("Restriction load node initialized, loading from: %s",
                 load_path_.c_str());
    }

   private:
    // 定时器回调函数
    void loadTimerCallback(const ros::TimerEvent&) {
        RestrictionData data;
        // 尝试从文件加载数据
        if (data.loadFromFile(load_path_)) {
            map_restriction_plugin::RestrictionArray msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "map";

            // 将加载的数据转换为消息
            for (const auto& restriction : data.getRestrictions()) {
                msg.restrictions.push_back(restriction);
            }

            // 发布消息
            restriction_pub_.publish(msg);
            ROS_INFO("Successfully loaded and published restrictions from: %s",
                     load_path_.c_str());
        }
    }

    ros::NodeHandle nh_;
    ros::Publisher restriction_pub_;
    ros::Timer load_timer_;
    std::string load_path_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "restriction_load_node");
    LoadNode node;
    ros::spin();
    return 0;
}
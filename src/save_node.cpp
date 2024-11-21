// src/save_node.cpp
#include <ros/ros.h>

#include "map_restriction_plugin/RestrictionArray.h"
#include "map_restriction_plugin/restriction_data.h"

class SaveNode {
   public:
    SaveNode() : nh_("~") {
        // 获取保存路径参数
        if (!nh_.getParam("save_path", save_path_)) {
            save_path_ = "restrictions.yaml";
            ROS_WARN("No save_path specified, using default: %s",
                     save_path_.c_str());
        }

        // 订阅禁区消息
        restriction_sub_ = nh_.subscribe("/map_restrictions", 1,
                                         &SaveNode::restrictionCallback, this);
        ROS_INFO("Restriction save node initialized, saving to: %s",
                 save_path_.c_str());
    }

   private:
    // 禁区消息回调函数
    void restrictionCallback(
        const map_restriction_plugin::RestrictionArray::ConstPtr& msg) {
        RestrictionData data;
        // 将消息中的禁区数据添加到数据对象
        for (const auto& restriction : msg->restrictions) {
            data.addRestriction(restriction);
        }

        // 保存到文件
        if (data.saveToFile(save_path_)) {
            ROS_INFO("Successfully saved restrictions to: %s",
                     save_path_.c_str());
        } else {
            ROS_ERROR("Failed to save restrictions to: %s", save_path_.c_str());
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber restriction_sub_;
    std::string save_path_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "restriction_save_node");
    SaveNode node;
    ros::spin();
    return 0;
}
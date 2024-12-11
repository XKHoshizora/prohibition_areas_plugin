// prohibition_areas_layer_helper.h
#ifndef PROHIBITION_AREAS_LAYER_HELPER_H
#define PROHIBITION_AREAS_LAYER_HELPER_H

#include <XmlRpc/XmlRpcValue.h>
#include <ros/ros.h>
#include <string>

namespace prohibition_areas_layer
{

class ProhibitionAreasHelper
{
public:
    /**
     * 将新格式的禁区定义转换为旧格式
     * 新格式:
     * - name: "area1"
     *   frame_id: map
     *   points: [[x1,y1,z1], [x2,y2,z2],...]
     *
     * 旧格式:
     * [[x1,y1], [x2,y2], ...]
     */
    static bool convertFormat(ros::NodeHandle* nhandle, const std::string& param) {
        XmlRpc::XmlRpcValue param_yaml;
        std::string converted_param = param + "_converted";

        try {
            // 读取原始参数
            if (!nhandle->getParam(param, param_yaml)) {
                ROS_ERROR_STREAM("Cannot read " << param << " from parameter server");
                return false;
            }

            // 检查是否是数组
            if (param_yaml.getType() != XmlRpc::XmlRpcValue::TypeArray) {
                ROS_ERROR_STREAM(param << " must be an array");
                return false;
            }

            // 创建新的XmlRpcValue来存储转换后的格式
            XmlRpc::XmlRpcValue converted;
            converted.setSize(0);  // 创建一个空数组

            // 遍历每个禁区
            for (int i = 0; i < param_yaml.size(); ++i) {
                XmlRpc::XmlRpcValue& area = param_yaml[i];

                // 检查必要的字段
                if (!area.hasMember("points") || !area.hasMember("name")) {
                    ROS_WARN_STREAM("Area " << i << " missing required fields, skipping");
                    continue;
                }

                // 获取点数组
                XmlRpc::XmlRpcValue& points = area["points"];
                if (points.getType() != XmlRpc::XmlRpcValue::TypeArray) {
                    ROS_WARN_STREAM("Points for area " << i << " must be an array");
                    continue;
                }

                // 创建这个区域的点数组
                XmlRpc::XmlRpcValue area_points;
                area_points.setSize(points.size());

                // 复制点数据
                for (int j = 0; j < points.size(); ++j) {
                    XmlRpc::XmlRpcValue& point = points[j];
                    if (point.getType() != XmlRpc::XmlRpcValue::TypeArray || point.size() < 2) {
                        ROS_WARN_STREAM("Invalid point format in area " << i << ", point " << j);
                        continue;
                    }
                    area_points[j] = point;
                }

                // 将这个区域添加到转换后的数组中
                converted.arrayPush(area_points);

                // 记录区域名称到参数服务器（可选）
                std::string area_name = area["name"];
                std::string area_param = param + "/area_" + std::to_string(i) + "_name";
                nhandle->setParam(area_param, area_name);
            }

            // 将转换后的格式设置到参数服务器
            nhandle->setParam(converted_param, converted);

            ROS_INFO("Successfully converted prohibition areas format");
            return true;

        } catch (XmlRpc::XmlRpcException& e) {
            ROS_ERROR_STREAM("Error converting prohibition areas format: " << e.getMessage());
            return false;
        }
    }
};

}  // namespace prohibition_areas_layer

#endif // PROHIBITION_AREAS_LAYER_HELPER_H
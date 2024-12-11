// prohibition_areas_layer_helper.h
#ifndef PROHIBITION_AREAS_LAYER_HELPER_H
#define PROHIBITION_AREAS_LAYER_HELPER_H

#include <xmlrpcpp/XmlRpcValue.h>
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

            // 首先收集有效的区域到一个临时向量
            std::vector<XmlRpc::XmlRpcValue> valid_areas;

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

                // 将有效的区域添加到临时向量
                valid_areas.push_back(area_points);

                // 记录区域名称到参数服务器
                std::string area_name = area["name"];
                std::string area_param = param + "/area_" + std::to_string(i) + "_name";
                nhandle->setParam(area_param, area_name);
            }

            // 创建转换后的格式
            XmlRpc::XmlRpcValue converted;
            converted.setSize(valid_areas.size());
            for (size_t i = 0; i < valid_areas.size(); ++i) {
                converted[i] = valid_areas[i];
            }

            // 将转换后的格式设置到参数服务器
            nhandle->setParam(converted_param, converted);

            ROS_INFO_STREAM("Successfully converted " << valid_areas.size() << " prohibition areas");
            return true;

        } catch (XmlRpc::XmlRpcException& e) {
            ROS_ERROR_STREAM("Error converting prohibition areas format: " << e.getMessage());
            return false;
        }
    }
};

}  // namespace prohibition_areas_layer

#endif // PROHIBITION_AREAS_LAYER_HELPER_H
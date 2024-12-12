// prohibition_areas_layer_helper.h
#ifndef PROHIBITION_AREAS_LAYER_HELPER_H
#define PROHIBITION_AREAS_LAYER_HELPER_H

#include <xmlrpcpp/XmlRpcValue.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <fstream>

namespace prohibition_areas_layer
{

class ProhibitionAreasHelper
{
public:
    /* Load prohibition areas from YAML file and convert to XmlRpcValue format.
     * @param file_path: Path to the YAML file containing prohibition areas.
     * @param nh: NodeHandle to set the prohibition areas parameter.
     * @return True if the file was successfully loaded and converted, false otherwise.
     */
    static bool loadAndConvertYamlFile(const std::string& file_path, ros::NodeHandle& nh) {
        try {
            // 读取YAML文件
            std::ifstream fin(file_path.c_str());
            if (!fin.good()) {
                ROS_ERROR_STREAM("Cannot open file: " << file_path);
                return false;
            }

            YAML::Node yaml_node = YAML::LoadFile(file_path);
            if (!yaml_node.IsSequence() && !yaml_node.IsNull()) {
                ROS_ERROR("Invalid YAML format: root must be a sequence or empty");
                return false;
            }

            // 如果文件为空或者是一个空序列，设置一个空的禁区列表
            if (yaml_node.IsNull() || yaml_node.size() == 0) {
                XmlRpc::XmlRpcValue empty_list;
                empty_list.setSize(0);
                nh.setParam("prohibition_areas", empty_list);
                return true;
            }

            // 转换为XmlRpcValue格式
            XmlRpc::XmlRpcValue areas;
            areas.setSize(yaml_node.size());

            for (size_t i = 0; i < yaml_node.size(); ++i) {
                const auto& area = yaml_node[i];
                if (!area["points"] || !area["name"]) {
                    ROS_WARN_STREAM("Skipping area " << i << ": missing required fields");
                    continue;
                }

                const auto& points = area["points"];
                XmlRpc::XmlRpcValue area_points;
                area_points.setSize(points.size());

                for (size_t j = 0; j < points.size(); ++j) {
                    const auto& point = points[j];
                    if (!point.IsSequence() || point.size() < 2) {
                        ROS_WARN_STREAM("Invalid point format in area " << i);
                        continue;
                    }

                    XmlRpc::XmlRpcValue point_array;
                    point_array.setSize(2);
                    point_array[0] = point[0].as<double>();
                    point_array[1] = point[1].as<double>();
                    area_points[j] = point_array;
                }

                areas[i] = area_points;

                // 保存区域名称作为额外参数
                std::string area_name = area["name"].as<std::string>();
                std::string area_param = "prohibition_areas/area_" + std::to_string(i) + "_name";
                nh.setParam(area_param, area_name);
            }

            // 将转换后的数据设置到参数服务器
            nh.setParam("prohibition_areas", areas);
            return true;

        } catch (const YAML::Exception& e) {
            ROS_ERROR_STREAM("YAML parsing error: " << e.what());
            return false;
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error processing prohibition areas: " << e.what());
            return false;
        }
    }

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
    static bool convertFormat(ros::NodeHandle& nhandle, const std::string& param) {
        XmlRpc::XmlRpcValue param_yaml;
        std::string converted_param = param + "_converted";

        try {
            // 读取原始参数
            if (!nhandle.getParam(param, param_yaml)) {
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

                bool valid_area = true;
                // 复制点数据，只保留x和y坐标
                for (int j = 0; j < points.size(); ++j) {
                    XmlRpc::XmlRpcValue& point = points[j];
                    if (point.getType() != XmlRpc::XmlRpcValue::TypeArray || point.size() < 2) {
                        ROS_WARN_STREAM("Invalid point format in area " << i << ", point " << j);
                        valid_area = false;
                        break;
                    }

                    // 创建新的只包含x,y的点
                    XmlRpc::XmlRpcValue xy_point;
                    xy_point.setSize(2);  // 只设置两个值：x和y

                    try {
                        // 确保转换为double类型
                        double x = point[0].getType() == XmlRpc::XmlRpcValue::TypeInt ?
                            static_cast<double>(static_cast<int>(point[0])) :
                            static_cast<double>(point[0]);

                        double y = point[1].getType() == XmlRpc::XmlRpcValue::TypeInt ?
                            static_cast<double>(static_cast<int>(point[1])) :
                            static_cast<double>(point[1]);

                        xy_point[0] = x;
                        xy_point[1] = y;

                        area_points[j] = xy_point;

                    } catch (const XmlRpc::XmlRpcException& e) {
                        ROS_ERROR_STREAM("Error converting point coordinates: " << e.getMessage());
                        valid_area = false;
                        break;
                    }
                }

                if (valid_area) {
                    // 将有效的区域添加到临时向量
                    valid_areas.push_back(area_points);

                    // 记录区域名称到参数服务器
                    std::string area_name = area["name"];
                    std::string area_param = param + "/area_" + std::to_string(valid_areas.size()-1) + "_name";
                    nhandle.setParam(area_param, area_name);

                    ROS_INFO_STREAM("Successfully converted area: " << area_name <<
                                  " with " << points.size() << " points");
                }
            }

            if (valid_areas.empty()) {
                ROS_ERROR("No valid areas found after conversion");
                return false;
            }

            // 创建转换后的格式
            XmlRpc::XmlRpcValue converted;
            converted.setSize(valid_areas.size());
            for (size_t i = 0; i < valid_areas.size(); ++i) {
                converted[i] = valid_areas[i];
            }

            // 将转换后的格式设置到参数服务器
            nhandle.setParam(converted_param, converted);

            ROS_INFO_STREAM("Successfully converted " << valid_areas.size() <<
                          " prohibition areas with XY coordinates only");
            return true;

        } catch (const XmlRpc::XmlRpcException& e) {
            ROS_ERROR_STREAM("Error converting prohibition areas format: " << e.getMessage());
            return false;
        }
    }
};

}  // namespace prohibition_areas_layer

#endif // PROHIBITION_AREAS_LAYER_HELPER_H
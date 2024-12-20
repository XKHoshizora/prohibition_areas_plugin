/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Stephan Kurzawe
 * Modified by YIBO SUN
 *********************************************************************/

#include <pluginlib/class_list_macros.h>
#include <prohibition_areas_plugin/prohibition_areas_layer/prohibition_areas_layer.h>
#include <prohibition_areas_plugin/prohibition_areas_layer/prohibition_areas_layer_helper.h>

PLUGINLIB_EXPORT_CLASS(prohibition_areas_layer::ProhibitionAreasLayer,
                       costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace prohibition_areas_layer {

ProhibitionAreasLayer::ProhibitionAreasLayer() : _dsrv(NULL) {}

ProhibitionAreasLayer::~ProhibitionAreasLayer() {
    if (_dsrv != NULL) delete _dsrv;
}

void ProhibitionAreasLayer::onInitialize() {
    nh = ros::NodeHandle("~/" + name_);
    current_ = true;

    _dsrv = new dynamic_reconfigure::Server<ProhibitionAreasLayerConfig>(nh);
    dynamic_reconfigure::Server<ProhibitionAreasLayerConfig>::CallbackType cb =
        boost::bind(&ProhibitionAreasLayer::reconfigureCB, this, _1, _2);
    _dsrv->setCallback(cb);

    // get a pointer to the layered costmap and save resolution
    costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();
    _costmap_resolution = costmap->getResolution();

    // set initial bounds
    _min_x = _min_y = _max_x = _max_y = 0;

    // reading the prohibition areas out of the namespace of this plugin!
    // e.g.: "move_base/global_costmap/prohibition_layer/prohibition_areas"

    // 获取配置的文件路径
    bool has_custom_path = nh.getParam("data_file", prohibition_file_path_);

    if (!has_custom_path) {
        // 如果没有配置文件路径，使用默认路径
        std::string pkg_path = ros::package::getPath("prohibition_areas_plugin");
        prohibition_file_path_ = pkg_path + "/prohibition_areas/prohibition_areas.yaml";
        ROS_INFO_STREAM("Using default prohibition areas file: " << prohibition_file_path_);
    } else {
        ROS_INFO_STREAM("Using configured prohibition areas file: " << prohibition_file_path_);
    }

    // 检查文件是否存在
    std::ifstream f(prohibition_file_path_.c_str());
    if (!f.good()) {
        ROS_WARN_STREAM("Prohibition areas file not found: " << prohibition_file_path_);
        // 如果是用户配置的文件不存在，返回错误
        if (has_custom_path) {
            ROS_ERROR("Configured prohibition areas file does not exist!");
            // 可以选择抛出异常或设置错误标志
            return;
        }
    }

    // 保存参数名
    param_name_ = "prohibition_areas";

    // 首次加载禁区数据
    loadProhibitionAreas();

    // 订阅更新消息
    update_sub_ = nh.subscribe("/prohibition_areas_update", 1,
                              &ProhibitionAreasLayer::updateCallback, this);

    // if (!parseProhibitionListFromYaml(&nh, params))
    //   ROS_ERROR_STREAM("Reading prohibition areas from '" <<
    //   nh.getNamespace() << "/" << params << "' failed!");

    _fill_polygons = true;
    nh.param("fill_polygons", _fill_polygons, _fill_polygons);

    // compute map bounds for the current set of prohibition areas.
    computeMapBounds();

    ROS_INFO("ProhibitionAreasLayer initialized.");
}

// 专门用于加载禁区数据的函数
bool ProhibitionAreasLayer::loadProhibitionAreas() {
    // 直接从文件加载并转换到参数服务器
    if (!ProhibitionAreasHelper::loadAndConvertYamlFile(prohibition_file_path_, nh)) {
        ROS_ERROR_STREAM("Failed to load prohibition areas from file: " << prohibition_file_path_);
        return false;
    }

    // 尝试从参数服务器读取转换后的数据
    if (!parseProhibitionListFromYaml(nh, param_name_)) {
        ROS_ERROR_STREAM("Reading prohibition areas from parameter server failed!");
        return false;
    }

    return true;
}

void ProhibitionAreasLayer::updateCallback(const std_msgs::Empty::ConstPtr& msg) {
    // 加载新的禁区数据
    if (loadProhibitionAreas()) {
        // 更新边界
        updateBounds(0, 0, 0, &_min_x, &_min_y, &_max_x, &_max_y);
        // 重新计算地图边界
        computeMapBounds();
    }
}

void ProhibitionAreasLayer::reconfigureCB(ProhibitionAreasLayerConfig &config,
                                          uint32_t level) {
    enabled_ = config.enabled;
    _fill_polygons = config.fill_polygons;
}

void ProhibitionAreasLayer::updateBounds(double robot_x, double robot_y,
                                         double robot_yaw, double *min_x,
                                         double *min_y, double *max_x,
                                         double *max_y) {
    if (!enabled_) return;

    std::lock_guard<std::mutex> l(_data_mutex);

    if (_prohibition_points.empty() && _prohibition_polygons.empty()) return;

    *min_x = std::min(*min_x, _min_x);
    *min_y = std::min(*min_y, _min_y);
    *max_x = std::max(*max_x, _max_x);
    *max_y = std::max(*max_y, _max_y);
}

void ProhibitionAreasLayer::updateCosts(costmap_2d::Costmap2D &master_grid,
                                        int min_i, int min_j, int max_i,
                                        int max_j) {
    if (!enabled_) return;

    std::lock_guard<std::mutex> l(_data_mutex);

    // set costs of polygons
    for (int i = 0; i < _prohibition_polygons.size(); ++i) {
        setPolygonCost(master_grid, _prohibition_polygons[i], LETHAL_OBSTACLE,
                       min_i, min_j, max_i, max_j, _fill_polygons);
    }

    // set cost of points
    for (int i = 0; i < _prohibition_points.size(); ++i) {
        unsigned int mx;
        unsigned int my;
        if (master_grid.worldToMap(_prohibition_points[i].x,
                                   _prohibition_points[i].y, mx, my)) {
            master_grid.setCost(mx, my, LETHAL_OBSTACLE);
        }
    }
}

void ProhibitionAreasLayer::computeMapBounds() {
    std::lock_guard<std::mutex> l(_data_mutex);

    // reset bounds
    _min_x = _min_y = _max_x = _max_y = 0;

    // iterate polygons
    for (int i = 0; i < _prohibition_polygons.size(); ++i) {
        for (int j = 0; j < _prohibition_polygons.at(i).size(); ++j) {
            double px = _prohibition_polygons.at(i).at(j).x;
            double py = _prohibition_polygons.at(i).at(j).y;
            _min_x = std::min(px, _min_x);
            _min_y = std::min(py, _min_y);
            _max_x = std::max(px, _max_x);
            _max_y = std::max(py, _max_y);
        }
    }

    // iterate points
    for (int i = 0; i < _prohibition_points.size(); ++i) {
        double px = _prohibition_points.at(i).x;
        double py = _prohibition_points.at(i).y;
        _min_x = std::min(px, _min_x);
        _min_y = std::min(py, _min_y);
        _max_x = std::max(px, _max_x);
        _max_y = std::max(py, _max_y);
    }
}

void ProhibitionAreasLayer::setPolygonCost(
    costmap_2d::Costmap2D &master_grid,
    const std::vector<geometry_msgs::Point> &polygon, unsigned char cost,
    int min_i, int min_j, int max_i, int max_j, bool fill_polygon) {
    std::vector<PointInt> map_polygon;
    for (unsigned int i = 0; i < polygon.size(); ++i) {
        PointInt loc;
        master_grid.worldToMapNoBounds(polygon[i].x, polygon[i].y, loc.x,
                                       loc.y);
        map_polygon.push_back(loc);
    }

    std::vector<PointInt> polygon_cells;

    // get the cells that fill the polygon
    rasterizePolygon(map_polygon, polygon_cells, fill_polygon);

    // set the cost of those cells
    for (unsigned int i = 0; i < polygon_cells.size(); ++i) {
        int mx = polygon_cells[i].x;
        int my = polygon_cells[i].y;
        // check if point is outside bounds
        if (mx < min_i || mx >= max_i) continue;
        if (my < min_j || my >= max_j) continue;
        master_grid.setCost(mx, my, cost);
    }
}

void ProhibitionAreasLayer::polygonOutlineCells(
    const std::vector<PointInt> &polygon,
    std::vector<PointInt> &polygon_cells) {
    for (unsigned int i = 0; i < polygon.size() - 1; ++i) {
        raytrace(polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y,
                 polygon_cells);
    }
    if (!polygon.empty()) {
        unsigned int last_index = polygon.size() - 1;
        // we also need to close the polygon by going from the last point to the
        // first
        raytrace(polygon[last_index].x, polygon[last_index].y, polygon[0].x,
                 polygon[0].y, polygon_cells);
    }
}

void ProhibitionAreasLayer::raytrace(int x0, int y0, int x1, int y1,
                                     std::vector<PointInt> &cells) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    PointInt pt;
    pt.x = x0;
    pt.y = y0;
    int n = 1 + dx + dy;
    int x_inc = (x1 > x0) ? 1 : -1;
    int y_inc = (y1 > y0) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;

    for (; n > 0; --n) {
        cells.push_back(pt);

        if (error > 0) {
            pt.x += x_inc;
            error -= dy;
        } else {
            pt.y += y_inc;
            error += dx;
        }
    }
}

void ProhibitionAreasLayer::rasterizePolygon(
    const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells,
    bool fill) {
    // this implementation is a slighly modified version of
    // Costmap2D::convexFillCells(...)

    // we need a minimum polygon of a traingle
    if (polygon.size() < 3) return;

    // first get the cells that make up the outline of the polygon
    polygonOutlineCells(polygon, polygon_cells);

    if (!fill) return;

    // quick bubble sort to sort points by x
    PointInt swap;
    unsigned int i = 0;
    while (i < polygon_cells.size() - 1) {
        if (polygon_cells[i].x > polygon_cells[i + 1].x) {
            swap = polygon_cells[i];
            polygon_cells[i] = polygon_cells[i + 1];
            polygon_cells[i + 1] = swap;

            if (i > 0) --i;
        } else
            ++i;
    }

    i = 0;
    PointInt min_pt;
    PointInt max_pt;
    int min_x = polygon_cells[0].x;
    int max_x = polygon_cells[(int)polygon_cells.size() - 1].x;

    // walk through each column and mark cells inside the polygon
    for (int x = min_x; x <= max_x; ++x) {
        if (i >= (int)polygon_cells.size() - 1) break;

        if (polygon_cells[i].y < polygon_cells[i + 1].y) {
            min_pt = polygon_cells[i];
            max_pt = polygon_cells[i + 1];
        } else {
            min_pt = polygon_cells[i + 1];
            max_pt = polygon_cells[i];
        }

        i += 2;
        while (i < polygon_cells.size() && polygon_cells[i].x == x) {
            if (polygon_cells[i].y < min_pt.y)
                min_pt = polygon_cells[i];
            else if (polygon_cells[i].y > max_pt.y)
                max_pt = polygon_cells[i];
            ++i;
        }

        PointInt pt;
        // loop though cells in the column
        for (int y = min_pt.y; y < max_pt.y; ++y) {
            pt.x = x;
            pt.y = y;
            polygon_cells.push_back(pt);
        }
    }
}

// load prohibition positions out of the rosparam server
bool ProhibitionAreasLayer::parseProhibitionListFromYaml(
    ros::NodeHandle& nhandle, const std::string& param) {
    std::lock_guard<std::mutex> l(_data_mutex);
    std::unordered_map<std::string, geometry_msgs::Pose> map_out;

    XmlRpc::XmlRpcValue param_yaml;

    bool ret_val = true;

    if (nhandle.getParam(param, param_yaml)) {
        if (param_yaml.getType() ==
            XmlRpc::XmlRpcValue::TypeArray)  // list of goals
        {
            for (int i = 0; i < param_yaml.size(); ++i) {
                if (param_yaml[i].getType() == XmlRpc::XmlRpcValue::TypeArray) {
                    std::vector<geometry_msgs::Point> vector_to_add;

                    /* **************************************
                     * differ between points and polygons
                     * lines get to a polygon with the resolution
                     * of the costmap
                     **************************************** */

                    // add a point
                    if (param_yaml[i].size() == 1) {
                        geometry_msgs::Point point;
                        ret_val = getPoint(param_yaml[i][0], point);
                        _prohibition_points.push_back(point);
                    }
                    // add a line
                    else if (param_yaml[i].size() == 2) {
                        if (param_yaml[i][0].getType() ==
                                XmlRpc::XmlRpcValue::TypeDouble ||
                            param_yaml[i][0].getType() ==
                                XmlRpc::XmlRpcValue::TypeInt) {
                            // add a lonely point
                            geometry_msgs::Point point;
                            ret_val = getPoint(param_yaml[i], point);
                            _prohibition_points.push_back(point);
                        } else {
                            // add a line!
                            geometry_msgs::Point point_A;
                            ret_val = getPoint(param_yaml[i][0], point_A);
                            vector_to_add.push_back(point_A);

                            geometry_msgs::Point point_B;
                            ret_val = getPoint(param_yaml[i][1], point_B);
                            vector_to_add.push_back(point_B);

                            // calculate the normal vector for AB
                            geometry_msgs::Point point_N;
                            point_N.x = point_B.y - point_A.y;
                            point_N.y = point_A.x - point_B.x;

                            // get the absolute value of N to normalize and get
                            // it to the length of the costmap resolution
                            double abs_N =
                                sqrt(pow(point_N.x, 2) + pow(point_N.y, 2));
                            point_N.x = point_N.x / abs_N * _costmap_resolution;
                            point_N.y = point_N.y / abs_N * _costmap_resolution;

                            // calculate the new points to get a polygon which
                            // can be filled
                            geometry_msgs::Point point;
                            point.x = point_A.x + point_N.x;
                            point.y = point_A.y + point_N.y;
                            vector_to_add.push_back(point);

                            point.x = point_B.x + point_N.x;
                            point.y = point_B.y + point_N.y;
                            vector_to_add.push_back(point);

                            _prohibition_polygons.push_back(vector_to_add);
                        }
                    }
                    // add a point or add a polygon
                    else if (param_yaml[i].size() >= 3) {
                        // add a polygon with any number of points
                        for (int j = 0; j < param_yaml[i].size(); ++j) {
                            geometry_msgs::Point point;
                            ret_val = getPoint(param_yaml[i][j], point);
                            vector_to_add.push_back(point);
                        }
                        _prohibition_polygons.push_back(vector_to_add);
                    }
                } else {
                    ROS_ERROR_STREAM("Prohibition Layer:"
                                     << param << " with index " << i
                                     << " is not correct.");
                    ret_val = false;
                }
            }
        } else {
            ROS_ERROR_STREAM("Prohibition Layer: " << param
                                                   << "struct is not correct.");
            ret_val = false;
        }
    } else {
        ROS_ERROR_STREAM("Prohibition Layer: Cannot read "
                         << param << " from parameter server");
        ret_val = false;
    }
    return ret_val;
}

// get a point out of the XML Type into a geometry_msgs::Point
bool ProhibitionAreasLayer::getPoint(XmlRpc::XmlRpcValue &val,
                                     geometry_msgs::Point &point) {
    try {
        // check if there a two values for the coordinate
        if (val.getType() == XmlRpc::XmlRpcValue::TypeArray &&
            val.size() == 2) {
            auto convDouble = [](XmlRpc::XmlRpcValue &val) -> double {
                if (val.getType() ==
                    XmlRpc::XmlRpcValue::TypeInt)  // XmlRpc cannot cast int to
                                                   // double
                    return int(val);
                return val;  // if not double, an exception is thrown;
            };

            point.x = convDouble(val[0]);
            point.y = convDouble(val[1]);
            point.z = 0.0;
            return true;
        } else {
            ROS_ERROR_STREAM(
                "Prohibition_Layer: A point has to consist two values!");
            return false;
        }
    } catch (const XmlRpc::XmlRpcException &ex) {
        ROS_ERROR_STREAM(
            "Prohibition Layer: Cannot add current point: " << ex.getMessage());
        return false;
    }
}

}  // namespace prohibition_areas_layer

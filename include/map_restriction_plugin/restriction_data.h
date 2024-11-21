// include/map_restriction_plugin/restriction_data.h
#pragma once
#include <geometry_msgs/Point.h>

#include <vector>

#include "map_restriction_plugin/Restriction.h"

namespace map_restriction_plugin {

// 禁区数据管理类
class RestrictionData {
   public:
    // 添加新禁区
    void addRestriction(const Restriction& restriction);
    // 移除禁区
    void removeRestriction(size_t index);
    // 更新禁区
    void updateRestriction(size_t index, const Restriction& restriction);
    // 判断点是否在禁区内
    bool isPointRestricted(const geometry_msgs::Point& point) const;
    // 保存禁区到文件
    bool saveToFile(const std::string& filename) const;
    // 从文件加载禁区
    bool loadFromFile(const std::string& filename);
    // 获取所有禁区
    const std::vector<Restriction>& getRestrictions() const {
        return restrictions_;
    }

   private:
    std::vector<Restriction> restrictions_;
};

}  // namespace map_restriction_plugin
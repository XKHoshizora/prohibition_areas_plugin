// prohibition_areas_saver.h
#ifndef PROHIBITION_AREAS_SAVER_H
#define PROHIBITION_AREAS_SAVER_H

#include <geometry_msgs/Point.h>
#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

namespace prohibition_areas_tool {

struct ProhibitionArea {
    std::string name;
    std::string frame_id;
    std::vector<geometry_msgs::Point> points;
};

class ProhibitionAreasSaver {
   public:
    static bool saveToFile(const std::vector<ProhibitionArea>& areas,
                           const std::string& filename);
    static bool loadFromFile(std::vector<ProhibitionArea>& areas,
                             const std::string& filename);

   private:
    static YAML::Node pointToYaml(const geometry_msgs::Point& point);
    static geometry_msgs::Point yamlToPoint(const YAML::Node& node);
};

}  // end namespace prohibition_areas_tool

#endif  // PROHIBITION_AREAS_SAVER_H
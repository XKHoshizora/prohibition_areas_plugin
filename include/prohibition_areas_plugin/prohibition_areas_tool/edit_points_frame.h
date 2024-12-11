// edit_points_frame.h
#ifndef EDIT_POINTS_FRAME_H
#define EDIT_POINTS_FRAME_H

#include <geometry_msgs/Point.h>

#include <QWidget>
#include <string>
#include <vector>
#include <map>

#include "prohibition_areas_saver.h"

class QTreeWidget;
class QTreeWidgetItem;
class QPushButton;
class QVBoxLayout;
class QHBoxLayout;

namespace prohibition_areas_tool {

struct AreaData {
    std::string name;
    std::vector<geometry_msgs::Point> points;
};

class EditPointsFrame : public QWidget {
    Q_OBJECT

   public:
    explicit EditPointsFrame(const std::string& save_path, QWidget* parent = nullptr);
    ~EditPointsFrame() override;

    void setAreaPoints(const std::string& area_id,
                       const std::vector<geometry_msgs::Point>& points);
    void clearPoints();

   Q_SIGNALS:
    void pointSelected(int index);
    void pointDeleted(int index);
    void pointsModified();
    void areaSelected(const std::string& area_id);

   private Q_SLOTS:
    void onItemSelected(QTreeWidgetItem* item, int column);
    void onDeleteClicked();
    void onMoveUpClicked();
    void onMoveDownClicked();
    void onSaveClicked();
    void onLoadClicked();

   private:
    void updateTree();
    void setupUi();
    QString formatPoint(const geometry_msgs::Point& point);
    QTreeWidgetItem* findAreaItem(const std::string& area_id);
    bool isPointItem(QTreeWidgetItem* item);
    int getPointIndex(QTreeWidgetItem* item);

    QTreeWidget* tree_widget_;
    QPushButton* delete_button_;
    QPushButton* move_up_button_;
    QPushButton* move_down_button_;
    QPushButton* save_button_;
    QPushButton* load_button_;

    std::map<std::string, AreaData> areas_;  // 存储所有禁区数据
    std::string current_area_id_;            // 当前选中的禁区ID
    std::string save_path_;  // 保存路径
};

}  // end namespace prohibition_areas_tool

#endif  // EDIT_POINTS_FRAME_H
// edit_points_frame.h
#ifndef EDIT_POINTS_FRAME_H
#define EDIT_POINTS_FRAME_H

#include <geometry_msgs/Point.h>

#include <QWidget>
#include <string>
#include <vector>

#include "prohibition_areas_saver.h"

class QListWidget;
class QPushButton;
class QVBoxLayout;
class QHBoxLayout;

namespace prohibition_areas_tool {

class EditPointsFrame : public QWidget {
    Q_OBJECT
   public:
    EditPointsFrame(QWidget* parent = nullptr);
    ~EditPointsFrame() override;

    void setAreaPoints(const std::string& area_id,
                       const std::vector<geometry_msgs::Point>& points);
    void clearPoints();

   Q_SIGNALS:
    void pointSelected(int index);
    void pointDeleted(int index);
    void pointsModified();

   private Q_SLOTS:
    void onPointSelected(int index);
    void onDeleteClicked();
    void onMoveUpClicked();
    void onMoveDownClicked();
    void onSaveClicked();
    void onLoadClicked();

   private:
    void updatePointList();
    void setupUi();

    QListWidget* points_list_;
    QPushButton* delete_button_;
    QPushButton* move_up_button_;
    QPushButton* move_down_button_;
    QPushButton* save_button_;
    QPushButton* load_button_;

    std::string current_area_id_;
    std::vector<geometry_msgs::Point> points_;
};

}  // end namespace prohibition_areas_tool

#endif  // EDIT_POINTS_FRAME_H
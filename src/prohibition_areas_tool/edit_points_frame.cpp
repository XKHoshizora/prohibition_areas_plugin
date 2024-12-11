// edit_points_frame.cpp
#include <prohibition_areas_plugin/prohibition_areas_tool/edit_points_frame.h>
#include <prohibition_areas_plugin/prohibition_areas_tool/prohibition_areas_saver.h>

#include <QHBoxLayout>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <ros/ros.h>
#include <algorithm>
#include <sstream>

namespace prohibition_areas_tool {

EditPointsFrame::EditPointsFrame(QWidget* parent)
    : QWidget(parent),
      points_list_(nullptr),
      delete_button_(nullptr),
      move_up_button_(nullptr),
      move_down_button_(nullptr) {
    setupUi();
}

EditPointsFrame::~EditPointsFrame() {}

void EditPointsFrame::setupUi() {
    QVBoxLayout* layout = new QVBoxLayout;

    // 点列表
    points_list_ = new QListWidget;
    layout->addWidget(points_list_);

    // 按钮布局
    QHBoxLayout* button_layout = new QHBoxLayout;

    delete_button_ = new QPushButton("Delete");
    move_up_button_ = new QPushButton("Move Up");
    move_down_button_ = new QPushButton("Move Down");

    // 添加文件操作按钮
    QHBoxLayout* file_layout = new QHBoxLayout;
    save_button_ = new QPushButton("Save Areas");
    load_button_ = new QPushButton("Load Areas");

    button_layout->addWidget(delete_button_);
    button_layout->addWidget(move_up_button_);
    button_layout->addWidget(move_down_button_);

    layout->addLayout(button_layout);

    file_layout->addWidget(save_button_);
    file_layout->addWidget(load_button_);

    layout->addLayout(file_layout);

    setLayout(layout);

    // 连接信号和槽
    connect(points_list_, &QListWidget::currentRowChanged, this,
            &EditPointsFrame::onPointSelected);
    connect(delete_button_, &QPushButton::clicked, this,
            &EditPointsFrame::onDeleteClicked);
    connect(move_up_button_, &QPushButton::clicked, this,
            &EditPointsFrame::onMoveUpClicked);
    connect(move_down_button_, &QPushButton::clicked, this,
            &EditPointsFrame::onMoveDownClicked);
    connect(save_button_, &QPushButton::clicked, this,
            &EditPointsFrame::onSaveClicked);
    connect(load_button_, &QPushButton::clicked, this,
            &EditPointsFrame::onLoadClicked);

    // 初始状态设置
    delete_button_->setEnabled(false);
    move_up_button_->setEnabled(false);
    move_down_button_->setEnabled(false);

    setMinimumWidth(200);
}

void EditPointsFrame::setAreaPoints(
    const std::string& area_id,
    const std::vector<geometry_msgs::Point>& points) {
    current_area_id_ = area_id;
    points_ = points;
    updatePointList();
}

void EditPointsFrame::clearPoints() {
    points_.clear();
    current_area_id_.clear();
    updatePointList();
}

void EditPointsFrame::updatePointList() {
    points_list_->clear();

    for (size_t i = 0; i < points_.size(); ++i) {
        std::stringstream ss;
        ss << "Point " << i + 1 << " (" << points_[i].x << ", " << points_[i].y
           << ")";
        points_list_->addItem(ss.str().c_str());
    }

    // 更新按钮状态
    bool has_points = !points_.empty();
    delete_button_->setEnabled(has_points);
    move_up_button_->setEnabled(has_points && points_list_->currentRow() > 0);
    move_down_button_->setEnabled(has_points &&
                                  points_list_->currentRow() <
                                      static_cast<int>(points_.size()) - 1);
}

void EditPointsFrame::onPointSelected(int index) {
    bool valid_index = index >= 0 && index < static_cast<int>(points_.size());
    delete_button_->setEnabled(valid_index);
    move_up_button_->setEnabled(valid_index && index > 0);
    move_down_button_->setEnabled(valid_index &&
                                  index < static_cast<int>(points_.size()) - 1);

    if (valid_index) {
        Q_EMIT pointSelected(index);
    }
}

void EditPointsFrame::onDeleteClicked() {
    int index = points_list_->currentRow();
    if (index >= 0 && index < static_cast<int>(points_.size())) {
        points_.erase(points_.begin() + index);
        updatePointList();
        Q_EMIT pointDeleted(index);
        Q_EMIT pointsModified();
    }
}

void EditPointsFrame::onMoveUpClicked() {
    int index = points_list_->currentRow();
    if (index > 0 && index < static_cast<int>(points_.size())) {
        std::swap(points_[index], points_[index - 1]);
        updatePointList();
        points_list_->setCurrentRow(index - 1);
        Q_EMIT pointsModified();
    }
}

void EditPointsFrame::onMoveDownClicked() {
    int index = points_list_->currentRow();
    if (index >= 0 && index < static_cast<int>(points_.size()) - 1) {
        std::swap(points_[index], points_[index + 1]);
        updatePointList();
        points_list_->setCurrentRow(index + 1);
        Q_EMIT pointsModified();
    }
}

void EditPointsFrame::onSaveClicked() {
    QString filename =
        QFileDialog::getSaveFileName(this, tr("Save Prohibition Areas"), "",
                                     tr("YAML files (*.yaml);;All Files (*)"));

    if (filename.isEmpty()) return;

    std::vector<ProhibitionArea> areas;
    ProhibitionArea area;
    area.name = current_area_id_;
    area.frame_id = "map";  // 使用固定的frame_id
    area.points = points_;
    areas.push_back(area);

    if (ProhibitionAreasSaver::saveToFile(areas, filename.toStdString())) {
        QMessageBox::information(this, tr("Success"),
                                 tr("Prohibition areas saved successfully."));
    } else {
        QMessageBox::critical(this, tr("Error"),
                              tr("Failed to save prohibition areas."));
    }
}

void EditPointsFrame::onLoadClicked() {
    QString filename = QFileDialog::getOpenFileName(this,
        tr("Load Prohibition Areas"), "",
        tr("YAML files (*.yaml);;All Files (*)"));

    if (filename.isEmpty()) return;

    std::vector<ProhibitionArea> areas;
    if (ProhibitionAreasSaver::loadFromFile(areas, filename.toStdString())) {
        if (areas.empty()) {
            QMessageBox::warning(this, tr("Warning"),
                tr("No valid areas found in file."));
            return;
        }

        // 验证frame_id
        std::string expected_frame = "map"; // 或从配置获取
        for (const auto& area : areas) {
            if (area.frame_id != expected_frame) {
                QMessageBox::warning(this, tr("Warning"),
                    tr("Frame ID mismatch: expected '%1', got '%2' for area '%3'")
                    .arg(QString::fromStdString(expected_frame))
                    .arg(QString::fromStdString(area.frame_id))
                    .arg(QString::fromStdString(area.name)));
                continue;
            }

            if (area.points.size() < 3) {
                ROS_WARN_STREAM("Skipping area '" << area.name
                    << "': insufficient points");
                continue;
            }

            // 更新当前区域
            current_area_id_ = area.name;
            points_ = area.points;
            updatePointList();
            Q_EMIT pointsModified();
        }
    } else {
        QMessageBox::critical(this, tr("Error"),
            tr("Failed to load prohibition areas."));
    }
}

}  // end namespace prohibition_areas_tool
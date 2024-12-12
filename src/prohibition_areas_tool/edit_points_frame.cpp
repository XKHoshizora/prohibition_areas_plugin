// edit_points_frame.cpp
#include <prohibition_areas_plugin/prohibition_areas_tool/edit_points_frame.h>
#include <prohibition_areas_plugin/prohibition_areas_tool/prohibition_areas_saver.h>

#include <QTreeWidget>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <QInputDialog>
#include <ros/ros.h>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <sys/stat.h>

namespace prohibition_areas_tool {

EditPointsFrame::EditPointsFrame(const std::string& save_path, QWidget* parent)
    : QWidget(parent),
      tree_widget_(nullptr),
      delete_button_(nullptr),
      move_up_button_(nullptr),
      move_down_button_(nullptr),
      save_button_(nullptr),
      load_button_(nullptr),
      save_path_(save_path) {
    setupUi();
}

EditPointsFrame::~EditPointsFrame() = default;

void EditPointsFrame::setupUi() {
    // 设置窗口属性，保持在最前面
    setWindowFlags(Qt::Window | Qt::WindowStaysOnTopHint);
    setWindowTitle("Prohibition Areas Editor");

    QVBoxLayout* layout = new QVBoxLayout;

    // 创建树形控件
    tree_widget_ = new QTreeWidget;
    tree_widget_->setHeaderLabels({"Prohibition Areas"});
    tree_widget_->setExpandsOnDoubleClick(true);
    tree_widget_->setSelectionMode(QAbstractItemView::SingleSelection);
    tree_widget_->header()->setSectionResizeMode(QHeaderView::ResizeToContents);
    layout->addWidget(tree_widget_);

    // 按钮布局
    QHBoxLayout* button_layout = new QHBoxLayout;
    delete_button_ = new QPushButton("Delete");
    move_up_button_ = new QPushButton("Move Up");
    move_down_button_ = new QPushButton("Move Down");

    button_layout->addWidget(delete_button_);
    button_layout->addWidget(move_up_button_);
    button_layout->addWidget(move_down_button_);

    // 文件操作按钮
    QHBoxLayout* file_layout = new QHBoxLayout;
    save_button_ = new QPushButton("Save Areas");
    load_button_ = new QPushButton("Load Areas");

    file_layout->addWidget(save_button_);
    file_layout->addWidget(load_button_);

    layout->addLayout(button_layout);
    layout->addLayout(file_layout);

    setLayout(layout);

    // 连接信号和槽
    connect(tree_widget_, &QTreeWidget::itemSelectionChanged, [this]() {
        QTreeWidgetItem* item = tree_widget_->currentItem();
        if (item) {
            onItemSelected(item, 0);
        }
    });
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

    setMinimumWidth(300);
}

QString EditPointsFrame::formatPoint(const geometry_msgs::Point& point) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << "(" << point.x << ", " << point.y << ")";
    return QString::fromStdString(ss.str());
}

void EditPointsFrame::setAreaPoints(const std::string& area_id,
                                  const std::vector<geometry_msgs::Point>& points) {
    // 如果是新区域或更新已有区域
    AreaData& area = areas_[area_id];
    area.name = area_id;
    area.points = points;
    current_area_id_ = area_id;
    updateTree();
    Q_EMIT pointsModified();

    // 展开新添加的区域节点
    if (QTreeWidgetItem* item = findAreaItem(area_id)) {
        item->setExpanded(true);
        tree_widget_->setCurrentItem(item);
    }
}

void EditPointsFrame::clearPoints() {
    areas_.clear();
    current_area_id_.clear();
    updateTree();
    Q_EMIT pointsModified();
}

void EditPointsFrame::updateTree() {
    tree_widget_->clear();

    for (const auto& area_pair : areas_) {
        const AreaData& area = area_pair.second;

        // 创建区域项
        QTreeWidgetItem* area_item = new QTreeWidgetItem(tree_widget_);
        area_item->setText(0, QString::fromStdString(area.name));
        area_item->setFlags(area_item->flags() | Qt::ItemIsEditable);
        area_item->setData(0, Qt::UserRole, QString::fromStdString(area.name));

        // 添加点
        for (size_t i = 0; i < area.points.size(); ++i) {
            QTreeWidgetItem* point_item = new QTreeWidgetItem(area_item);
            point_item->setText(0, QString("Point %1 %2")
                .arg(i + 1)
                .arg(formatPoint(area.points[i])));
            point_item->setData(0, Qt::UserRole, static_cast<int>(i));
        }

        // 如果是当前选中的区域，展开它
        if (area.name == current_area_id_) {
            area_item->setExpanded(true);
            tree_widget_->setCurrentItem(area_item);
        }
    }

    // 更新按钮状态
    QTreeWidgetItem* current = tree_widget_->currentItem();
    bool has_selection = current != nullptr;
    bool is_point = current && current->parent() != nullptr;
    bool can_move_up = is_point && current->parent()->indexOfChild(current) > 0;
    bool can_move_down = is_point &&
        current->parent()->indexOfChild(current) < current->parent()->childCount() - 1;

    delete_button_->setEnabled(has_selection);
    move_up_button_->setEnabled(can_move_up);
    move_down_button_->setEnabled(can_move_down);
}

QTreeWidgetItem* EditPointsFrame::findAreaItem(const std::string& area_id) {
    for (int i = 0; i < tree_widget_->topLevelItemCount(); ++i) {
        QTreeWidgetItem* item = tree_widget_->topLevelItem(i);
        if (item->data(0, Qt::UserRole).toString().toStdString() == area_id) {
            return item;
        }
    }
    return nullptr;
}

bool EditPointsFrame::isPointItem(QTreeWidgetItem* item) {
    return item && item->parent() != nullptr;
}

int EditPointsFrame::getPointIndex(QTreeWidgetItem* item) {
    if (!isPointItem(item)) return -1;
    return item->data(0, Qt::UserRole).toInt();
}

void EditPointsFrame::onItemSelected(QTreeWidgetItem* item, int column) {
    if (!item) return;

    if (isPointItem(item)) {
        // 选中了点
        QTreeWidgetItem* area_item = item->parent();
        current_area_id_ = area_item->data(0, Qt::UserRole).toString().toStdString();
        int point_index = getPointIndex(item);
        Q_EMIT pointSelected(point_index);
    } else {
        // 选中了区域
        current_area_id_ = item->data(0, Qt::UserRole).toString().toStdString();
        Q_EMIT areaSelected(current_area_id_);
    }

    // 更新按钮状态
    bool is_point = isPointItem(item);
    bool can_move_up = is_point && item->parent()->indexOfChild(item) > 0;
    bool can_move_down = is_point &&
        item->parent()->indexOfChild(item) < item->parent()->childCount() - 1;

    delete_button_->setEnabled(true);
    move_up_button_->setEnabled(can_move_up);
    move_down_button_->setEnabled(can_move_down);
}

void EditPointsFrame::onDeleteClicked() {
    QTreeWidgetItem* item = tree_widget_->currentItem();
    if (!item) return;

    if (isPointItem(item)) {
        // 删除点
        QTreeWidgetItem* area_item = item->parent();
        std::string area_id = area_item->data(0, Qt::UserRole).toString().toStdString();
        int point_index = getPointIndex(item);

        if (areas_.count(area_id)) {
            auto& points = areas_[area_id].points;
            if (point_index >= 0 && point_index < points.size()) {
                points.erase(points.begin() + point_index);
                Q_EMIT pointDeleted(point_index);
                Q_EMIT pointsModified();
            }
        }
    } else {
        // 删除整个区域
        std::string area_id = item->data(0, Qt::UserRole).toString().toStdString();
        if (QMessageBox::question(this, "Confirm Delete",
                QString("Delete area '%1' and all its points?")
                    .arg(QString::fromStdString(area_id))) == QMessageBox::Yes) {
            areas_.erase(area_id);
            if (current_area_id_ == area_id) {
                current_area_id_.clear();
            }
            Q_EMIT pointsModified();
        }
    }

    updateTree();
}

void EditPointsFrame::onMoveUpClicked() {
    QTreeWidgetItem* item = tree_widget_->currentItem();
    if (!isPointItem(item)) return;

    QTreeWidgetItem* area_item = item->parent();
    std::string area_id = area_item->data(0, Qt::UserRole).toString().toStdString();
    int index = getPointIndex(item);

    if (index > 0 && areas_.count(area_id)) {
        auto& points = areas_[area_id].points;
        std::swap(points[index], points[index - 1]);
        Q_EMIT pointsModified();
        updateTree();
    }
}

void EditPointsFrame::onMoveDownClicked() {
    QTreeWidgetItem* item = tree_widget_->currentItem();
    if (!isPointItem(item)) return;

    QTreeWidgetItem* area_item = item->parent();
    std::string area_id = area_item->data(0, Qt::UserRole).toString().toStdString();
    int index = getPointIndex(item);

    if (areas_.count(area_id)) {
        auto& points = areas_[area_id].points;
        if (index >= 0 && index < points.size() - 1) {
            std::swap(points[index], points[index + 1]);
            Q_EMIT pointsModified();
            updateTree();
        }
    }
}

void EditPointsFrame::onSaveClicked() {
    if (areas_.empty()) {
        QMessageBox::warning(this, tr("Warning"),
            tr("No areas to save."));
        return;
    }

    std::vector<ProhibitionArea> existing_areas;
    // 先读取现有的禁区
    struct stat buffer;
    bool file_exists = (stat(save_path_.c_str(), &buffer) == 0);
    if (file_exists) {
        if (!ProhibitionAreasSaver::loadFromFile(existing_areas, save_path_)) {
            ROS_WARN("Failed to load existing areas when saving");
        }
    }

    // 更新或添加新的区域
    for (const auto& pair : areas_) {
        const AreaData& area_data = pair.second;
        if (area_data.points.size() < 3) continue;

        ProhibitionArea new_area;
        new_area.name = area_data.name;
        new_area.frame_id = "map";
        new_area.points = area_data.points;

        // 查找是否存在同名区域
        bool found = false;
        for (auto& existing_area : existing_areas) {
            if (existing_area.name == new_area.name) {
                existing_area = new_area;  // 更新已存在的区域
                found = true;
                break;
            }
        }

        if (!found) {
            existing_areas.push_back(new_area);  // 添加新区域
        }
    }

    if (existing_areas.empty()) {
        QMessageBox::warning(this, tr("Warning"),
            tr("No valid areas to save."));
        return;
    }

    if (ProhibitionAreasSaver::saveToFile(existing_areas, save_path_)) {
        QMessageBox::information(this, tr("Success"),
            tr("Prohibition areas saved successfully."));
        Q_EMIT pointsModified();  // 发出信号通知更新
    } else {
        QMessageBox::critical(this, tr("Error"),
            tr("Failed to save prohibition areas."));
    }
}

void EditPointsFrame::onLoadClicked() {
    QString filename = QFileDialog::getOpenFileName(
        this, tr("Load Prohibition Areas"), "",
        tr("YAML files (*.yaml);;All Files (*)"));

    if (filename.isEmpty()) return;

    std::vector<ProhibitionArea> loaded_areas;
    if (ProhibitionAreasSaver::loadFromFile(loaded_areas, filename.toStdString())) {
        if (loaded_areas.empty()) {
            QMessageBox::warning(this, tr("Warning"),
                               tr("No valid areas found in file."));
            return;
        }

        // 验证frame_id
        std::string expected_frame = "map";
        for (const auto& area : loaded_areas) {
            if (area.frame_id != expected_frame) {
                QMessageBox::warning(
                    this, tr("Warning"),
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

            // 更新区域数据
            current_area_id_ = area.name;
            areas_[current_area_id_].name = area.name;
            areas_[current_area_id_].points = area.points;
        }
        updateTree();
        Q_EMIT pointsModified();
    } else {
        QMessageBox::critical(this, tr("Error"),
                            tr("Failed to load prohibition areas."));
    }
}

std::vector<ProhibitionArea> EditPointsFrame::getAllAreas() const {
    std::vector<ProhibitionArea> result;
    for (const auto& pair : areas_) {
        ProhibitionArea area;
        area.name = pair.second.name;
        area.frame_id = "map";  // 或从配置获取
        area.points = pair.second.points;
        result.push_back(area);
    }
    return result;
}

}  // end namespace prohibition_areas_tool
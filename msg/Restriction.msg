# msg/Restriction.msg
# 禁区类型定义
uint8 RECTANGLE=0
uint8 POLYGON=1
uint8 CIRCLE=2

uint8 type               # 禁区类型
geometry_msgs/Point[] points  # 禁区顶点数组
float64 radius          # 圆形禁区半径
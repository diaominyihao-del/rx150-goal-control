# RX-150 ROS2 Goal Control

基于 ROS2 的 Interbotix RX-150 机械臂目标点控制项目。

## 功能

- 正运动学（齐次变换）
- 数值逆运动学（牛顿法 + 雅可比矩阵）
- 三次多项式轨迹生成
- 命令行输入目标点，机械臂平滑运动至目标点
- 关节角归一化与可达性检查

## 环境

- Ubuntu 22.04
- ROS2 Humble
- Interbotix ROS2 包
- Python 3
- numpy

## 运行

启动 RX-150 仿真：

```bash
ros2 launch interbotix_xsarm_descriptions xsarm_description.launch.py robot_model:=rx150 use_joint_pub_gui:=false
```

启动节点:
```bash
ros2 run rx150_goal_control goal_control
```

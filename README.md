# ROS 2 + MoveIt 2 六自由度机械臂控制保姆级教程

这是一个面向初学者的 ROS 2 Humble 与 MoveIt 2 机械臂仿真控制教程项目。仓库包含完整教程、配套图片资源，以及一个可构建的 ROS 2 工作空间源码，用于学习六自由度机械臂和夹爪从建模、规划到仿真执行的完整流程。

重要说明：本项目当前基于 `FakeSystem` 完成仿真控制验证，尚未接入真实机械臂硬件。文档中的硬件控制链路解释用于帮助理解 `MoveIt 2`、`ros2_control`、硬件接口和驱动之间的分工，不代表已经完成实机控制。

## 功能范围

- 六自由度机械臂 URDF/Xacro 建模
- 夹爪建模与 MoveIt 配置
- RViz 和 TF 可视化
- MoveIt 运动规划
- `ros2_control` 控制器
- `FakeSystem` 仿真硬件
- MoveIt C++ API Commander 节点
- 关节、位姿和夹爪控制话题
- 自定义消息接口

## 环境信息

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2 Humble
- C++
- RViz 2
- `ros2_control`

## 快速开始

完整步骤、代码演进和排错过程请阅读：

[查看完整保姆级教程](./基于ROS2与Moveit2的机械臂控制项目_开源教程版.md)

简化启动流程如下，具体命令和顺序以完整教程为准：

```bash
git clone https://github.com/<your-name>/ros2-moveit2-arm-control-tutorial.git
cd ros2-moveit2-arm-control-tutorial/ros2_ws
colcon build
source install/setup.bash
ros2 launch my_robot_bringup my_robot.launch.xml
```

另开终端后可启动 Commander 节点：

```bash
cd ros2-moveit2-arm-control-tutorial/ros2_ws
source install/setup.bash
ros2 run my_robot_commander_cpp commander_template
```

## 项目结构

```text
ros2-moveit2-arm-control-tutorial/
├── README.md
├── LICENSE
├── CONTRIBUTING.md
├── PUBLISHING_CHECKLIST.md
├── 基于ROS2与Moveit2的机械臂控制项目_开源教程版.md
├── 基于ROS2与Moveit2的机械臂控制项目.assets/
│   └── 教程图片和视频资源
└── ros2_ws/
    └── src/
        ├── my_robot_description/      # URDF/Xacro、RViz 和机器人描述启动文件
        ├── my_robot_moveit_config/    # MoveIt Setup Assistant 生成的规划配置
        ├── my_robot_bringup/          # 统一启动 robot_state_publisher、控制器、MoveIt 和 RViz
        ├── my_robot_commander_cpp/    # MoveIt C++ API Commander 控制节点
        └── my_robot_interfaces/       # 自定义消息 PoseCommand
```

教程 Markdown 与 `.assets` 文件夹必须保持同级，否则文档中的相对图片路径会失效。

## 控制链路

本项目的核心控制链路可以概括为：

```text
外部 ROS 2 节点
→ Commander
→ MoveIt 2
→ ros2_control Controller
→ FakeSystem
→ joint_states / TF
→ RViz
```

其中，MoveIt 2 负责运动规划；`ros2_control` 控制器负责接收轨迹并执行；`FakeSystem` 模拟硬件接口的命令接收和状态反馈；`robot_state_publisher` 根据 `/joint_states` 和 URDF 发布 TF，供 RViz 和 MoveIt 获取当前机器人状态。

## 已知限制

- 当前不是实机控制项目，尚未接入真实机械臂硬件。
- 建议在纯英文路径中创建 ROS 2 工作空间，例如 `~/ros2_ws`，以减少工具链和 overlay 环境带来的路径问题。
- 不同电脑上的 MoveIt、OMPL 和环境 overlay 可能需要自行排查。
- 教程中的代码按学习顺序逐步演进，前期代码不一定等同于 `ros2_ws/src` 中的最终源码。

## 贡献方式

欢迎提交 Issue 和 Pull Request。提交前请阅读 [CONTRIBUTING.md](./CONTRIBUTING.md)。

## 许可证

本项目使用 [MIT License](./LICENSE)。

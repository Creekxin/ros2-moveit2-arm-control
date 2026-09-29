// ============================================================================
// test_moveit.cpp —— MoveIt 2 C++ API 学习示例
//
// 用最基础的过程式写法（不封装类）演示 MoveGroupInterface 的 4 种控制方式：
//   1) 命名姿态目标  Named goal     —— 用 Setup Assistant 里存好的姿态名
//   2) 关节目标      Joint Goal     —— 直接给 6 个关节角
//   3) 末端位姿目标  Pose Goal      —— 给末端位置 + 姿态四元数
//   4) 笛卡尔路径    Cartesian Path —— 让末端按直线/折线走
//
// 当前文件只启用第 3、4 段，前两段保留为注释，方便对照学习。
// 更工程的写法见同目录 commander_template.cpp。
// ============================================================================

// ROS 2 C++ 客户端库：节点、执行器、日志等都来自这里
#include <rclcpp/rclcpp.hpp>
// MoveIt 2 最常用的高层接口：设置目标、规划、执行、算笛卡尔路径
#include <moveit/move_group_interface/move_group_interface.h>
// std::thread：把执行器放到单独线程里 spin
#include <thread>
// 带参考坐标系（header.frame_id）的位姿消息
#include <geometry_msgs/msg/pose_stamped.hpp>
// tf2 的四元数工具：把 RPY 欧拉角转成四元数
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv)
{
    // 初始化 ROS 2 客户端库（解析命令行参数，如 --ros-args）
    rclcpp::init(argc, argv);

    // 创建节点，名字为 test_moveit
    // MoveIt 通过该节点去连接后台的 move_group 节点
    auto node = std::make_shared<rclcpp::Node>("test_moveit");

    // ------------------------------------------------------------------
    // 【最容易踩的坑】MoveGroupInterface 的构造、plan()、execute() 内部
    // 都会发 service / action 请求并等待回调结果。
    // 如果没有执行器在 spin，这些调用会一直阻塞不返回。
    // 所以这里起一个单线程执行器，并在独立线程中 spin。
    // ------------------------------------------------------------------
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() {
        executor.spin();
    });

    // 创建机械臂规划组，对应 SRDF 中名为 "arm" 的 group
    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");

    // 最大速度缩放因子，有效范围 (0, 1]；1.0 表示直接用关节限位允许的最大速度
    arm.setMaxVelocityScalingFactor(1.0);
    // 最大加速度缩放因子，同上；1.0 表示用最大加速度
    arm.setMaxAccelerationScalingFactor(1.0);

    // 创建夹爪规划组，对应 SRDF 中名为 "gripper" 的 group
    // 本示例未使用夹爪，创建它只是为了说明"一个节点可以同时操作多个规划组"
    auto gripper = moveit::planning_interface::MoveGroupInterface(node, "gripper");


    // ==================================================================
    // 方式一：命名姿态目标（Named goal）
    //
    // 目标来自 MoveIt Setup Assistant 保存的命名姿态（SRDF 里的 group_state），
    // 本项目里有 pose_1、home 等。整段注释掉了，需要时取消注释即可。
    // ==================================================================
    // // Named goal
    // arm.setStartStateToCurrentState();  // 用机器人"当前状态"作为规划起点
    // arm.setNamedTarget("pose_1");       // 设置目标为命名姿态 pose_1

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;  // 用于接收规划结果

    // bool success1 = static_cast<bool>(arm.plan(plan1));  // 请求规划；plan() 返回错误码，可转 bool

    // if (success1)
    // {
    //     arm.execute(plan1);  // 规划成功 → 把轨迹发给控制器执行（阻塞到执行结束）
    // }
    // else
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Planning failed!");  // 失败时打印错误日志
    // }

    // // Named goal：回到 home 姿态，流程与上面完全一样，只是目标名不同
    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("home");

    // moveit::planning_interface::MoveGroupInterface::Plan plan2;

    // bool success2 = static_cast<bool>(arm.plan(plan2));

    // if (success2)
    // {
    //     arm.execute(plan2);
    // }
    // else
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Planning failed!");
    // }

    //--------------------------------------------------------------------------------

    // ==================================================================
    // 方式二：关节目标（Joint Goal）
    //
    // 直接给出 6 个关节的目标角度（单位：弧度），规划器负责规划到达过程。
    // 同样注释掉了，取消注释即可运行。
    // ==================================================================
    // //Joint Goal
    // std::vector<double> joints = { 1.5, 0.5, 0.0, 1.5, 0.0, -0.7 };  // 6 个关节角，单位弧度

    // arm.setStartStateToCurrentState();  // 起点 = 当前状态
    // arm.setJointValueTarget(joints);    // 设置关节角目标（会覆盖之前的位姿目标）

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;

    // // 这种写法与方式一等价，只是显式地拿 MoveItErrorCode 和 SUCCESS 比较
    // bool success1 =
    //     (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success1)
    // {
    //     arm.execute(plan1);
    // }


    //---------------------------------------------------------------------------------

    // ==================================================================
    // 方式三：末端位姿目标（Pose Goal）
    //
    // 只给末端执行器一个目标位置 + 目标姿态，
    // 中间各关节怎么转由 MoveIt 的规划器决定（可能绕路避障）。
    // ==================================================================

    // ---------- 3.1 构造目标姿态 ----------
    tf2::Quaternion q;
    q.setRPY(3.14, 0.0, 0.0);  // 绕 X 轴转约 180°，相当于末端"翻腕"朝下
    q = q.normalize();         // 归一化，保证四元数模长为 1

    geometry_msgs::msg::PoseStamped target_pose;
    // 注意：MoveGroupInterface 会忽略这里的 frame_id，
    // 真正的参考坐标系由 setPoseReferenceFrame() 决定，
    // 默认是机器人模型/规划坐标系（本项目为 base_link）。
    // 这里写 base_link 只是为了语义清晰。
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = 0.0;   // 位置单位：米
    target_pose.pose.position.y = -0.7;
    target_pose.pose.position.z = 0.4;
    target_pose.pose.orientation.x = q.getX();  // 姿态：把四元数 4 个分量填进消息
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();

    // ---------- 3.2 设置目标 → 规划 → 执行 ----------
    arm.setStartStateToCurrentState();  // 规划起点 = 机器人当前关节状态
    arm.setPoseTarget(target_pose);     // 设置末端位姿目标（会覆盖之前设置的关节目标）

    moveit::planning_interface::MoveGroupInterface::Plan plan1;  // 存放规划结果

    // plan() 返回 moveit::core::MoveItErrorCode，与 SUCCESS 比较即可判断成败
    bool success1 =
        (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success1)
    {
        arm.execute(plan1);  // 执行规划出的轨迹
    }

    // ==================================================================
    // 方式四：笛卡尔路径（Cartesian Path）
    //
    // 与方式三的区别：不是"只给终点、中间随便走"，
    // 而是让末端沿指定的路径点走直线/折线（这里是从当前位姿竖直抬升 0.2 m）。
    // 注意：起点用的是 getCurrentPose()，所以本段必须放在上面
    //       execute(plan1) 之后，否则取到的不是刚到位的那一点的位姿。
    // ==================================================================

    // Cartesian Path

    // 路径点列表：末端会依次经过这些位姿
    std::vector<geometry_msgs::msg::Pose> waypoints;

    // 取当前末端位姿作为起点（getCurrentPose() 返回 PoseStamped，这里只要 .pose）
    geometry_msgs::msg::Pose pose1 = arm.getCurrentPose().pose;
    pose1.position.z += 0.2;      // Z 方向 +0.2 m，姿态保持不变 → 近似竖直上升
    waypoints.push_back(pose1);   // 加入路径点列表

    // 生成的轨迹会写入这个对象（需依赖 moveit_msgs）
    moveit_msgs::msg::RobotTrajectory trajectory;

    // computeCartesianPath 参数逐个说明：
    //   waypoints          : 末端要依次经过的路径点
    //   eef_step = 0.01    : 末端步长 1 cm，越小轨迹越精细、计算量越大
    //   jump_threshold=0.0 : 关节空间跳变阈值，0 表示不做跳变过滤检查
    //   trajectory         : 输出参数，算出来的轨迹
    //   avoid_collisions=true : 沿路径做碰撞检查，躲不开就规划失败
    //   error_code = nullptr  : 可选的错误码输出，这里不需要接收
    // 返回值 fraction ∈ [0, 1]：实际走完的路径比例；返回 -1 表示出错
    double fraction = arm.computeCartesianPath(
        waypoints,
        0.01,
        0.0,
        trajectory,
        true,
        nullptr
    );

    // fraction == 1.0 表示整条路径都算出来了。
    // 提示：工程上更稳妥的写法是 if (fraction > 0.99)，
    // 可以避免浮点误差导致明明成功却判断为失败。
    if (fraction == 1.0)
    {
        arm.execute(trajectory);  // 执行笛卡尔轨迹
    }

    // 关闭 ROS 2（让 spin 退出）
    rclcpp::shutdown();
    // 等执行器线程真正结束，避免主线程先退出导致线程还在跑
    spinner.join();

    return 0;
}

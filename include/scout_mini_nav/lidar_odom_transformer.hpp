#ifndef SCOUT_MINI_NAV__LIDAR_ODOM_TRANSFORMER_HPP_
#define SCOUT_MINI_NAV__LIDAR_ODOM_TRANSFORMER_HPP_

#include <array>
#include <memory>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

/**
 * 将 FAST-LIO 的 IMU 位姿转换为车体位姿。
 *
 * 统一约定：T_A_B 把 B 系点变换到 A 系，p_A = R_A_B*p_B + t_A_B。
 * W=camera_init，O=odom，I=FAST-LIO body/IMU，L=lidar，B=base_link。
 *
 * FAST-LIO extrinsic_R/T 是雷达在 IMU 中的位姿：
 *   p_I = R_imu_lidar*p_L + t_imu_lidar。
 * 因而它是 lidar → IMU 的点坐标变换；若用 TF 表示，父帧为 IMU、子帧为 lidar。
 * 官方依据：https://github.com/hku-mars/FAST_LIO#3-directly-run
 * 源码依据：pointBodyToWorld 中 R_world_imu*(R_imu_lidar*p_L+t_imu_lidar)+t_world_imu。
 *
 * 安装外参 T_base_lidar 则满足 p_B = R_base_lidar*p_L + t_base_lidar。
 * 所需车体位姿：T_world_base = T_world_imu*T_imu_lidar*inverse(T_base_lidar)。
 * 首次有效车体位姿的位置和偏航定义 odom 原点与朝向，保留 W 的竖直方向。
 * T_odom_world 固定后，T_odom_base = T_odom_world*T_world_base。
 * 位姿与协方差均换算到 odom，采样时间保持原值；不是只改 frame_id。
 * 发布固定 odom → camera_init，使原始点云仍能正确变换到融合坐标系。
 * 默认不发布 odom → base_link 动态 TF，该变换由 EKF 统一发布。
 */
class LidarOdomTransformer : public rclcpp::Node {
 public:
  /// 启动时读取固定外参并建立订阅；无效矩阵、数组或坐标系参数会抛出异常。
  explicit LidarOdomTransformer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  using Matrix6d = Eigen::Matrix<double, 6, 6>;

  /// 读取并验证两组外参；t_imu_base_ 与 R_imu_base_ 是固定的车体 → IMU 变换。
  void loadParameters();

  /// 从完整三维 IMU 位姿构造车体位姿；检查输入并保持原始采样时间。
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /// 用首帧有效车体位姿建立固定世界基准；只初始化一次，不使用 EKF 反馈。
  void initializeOdomFrame(const nav_msgs::msg::Odometry &msg,
                           const Eigen::Matrix3d &R_world_base,
                           const Eigen::Vector3d &t_world_base);

  /// 换算位姿协方差，包含 IMU 到车体原点的杠杆臂及姿态误差坐标变换。
  Matrix6d convertPoseCovariance(
      const nav_msgs::msg::Odometry &msg, const Eigen::Matrix3d &R_world_imu,
      const Eigen::Matrix3d &R_world_base) const;

  std::string input_topic_;
  std::string output_topic_;
  std::string world_frame_;         // 输入世界坐标系，默认 camera_init。
  std::string output_world_frame_;  // 输出世界坐标系，默认 odom。
  std::string imu_frame_;    // 输入 child_frame_id，默认 body。
  std::string base_frame_;   // 输出 child_frame_id，默认 base_link。
  std::string covariance_order_;  // 当前 ROS2 分支误将角度协方差放在前 3 项。
  bool publish_tf_ = false;       // odom → base_link 动态 TF；融合时保持 false。
  bool publish_world_tf_ = true;  // 固定 odom → camera_init，用于原始点云。
  bool world_frame_initialized_ = false;
  // p_odom = R_odom_world_*p_world+t_odom_world_；首帧后保持不变。
  Eigen::Matrix3d R_odom_world_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_odom_world_ = Eigen::Vector3d::Zero();

  // extrinsic_R/T 原值，明确表示 lidar → IMU 的坐标变换。
  Eigen::Matrix3d R_imu_lidar_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_imu_lidar_ = Eigen::Vector3d::Zero();

  // base_link → lidar 静态 TF 的位姿值，表示 lidar → base_link 的坐标变换。
  Eigen::Matrix3d R_base_lidar_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_base_lidar_ = Eigen::Vector3d::Zero();

  // T_imu_base = T_imu_lidar * inverse(T_base_lidar)，表示车体 → IMU 的坐标变换。
  Eigen::Matrix3d R_imu_base_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_imu_base_ = Eigen::Vector3d::Zero();
  std::array<double, 6> pose_covariance_floor_{};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> world_tf_broadcaster_;
};

#endif  // SCOUT_MINI_NAV__LIDAR_ODOM_TRANSFORMER_HPP_

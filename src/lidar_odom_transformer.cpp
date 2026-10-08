#include "scout_mini_nav/lidar_odom_transformer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <vector>
#include <Eigen/Eigenvalues>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace {
/// 验证参数数组长度与数值，防止错误外参悄悄进入位姿转换。
void validateVector(const std::vector<double> &values, size_t count,
                    const std::string &name) {
  if (values.size() != count ||
      !std::all_of(values.begin(), values.end(),
                   [](double value) { return std::isfinite(value); })) {
    throw std::invalid_argument(name + " must contain " +
                                std::to_string(count) + " finite values");
  }
}

/// 返回叉乘矩阵 [v]x，用于计算旋转误差引起的原点位置误差。
Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d result;
  result << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return result;
}
}  // namespace

LidarOdomTransformer::LidarOdomTransformer(const rclcpp::NodeOptions &options)
    : Node("lidar_odom_transformer", options) {
  loadParameters();
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 20);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, 20, std::bind(&LidarOdomTransformer::odomCallback, this,
                                 std::placeholders::_1));
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }
  if (publish_world_tf_ && output_world_frame_ != world_frame_) {
    world_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
  }
  RCLCPP_INFO(get_logger(), "Converting %s (%s -> %s) to %s (%s -> %s); TF=%s",
              input_topic_.c_str(), world_frame_.c_str(), imu_frame_.c_str(),
              output_topic_.c_str(), output_world_frame_.c_str(), base_frame_.c_str(),
              publish_tf_ ? "true" : "false");
}

void LidarOdomTransformer::loadParameters() {
  input_topic_ = declare_parameter<std::string>("input_topic", "/Odometry");
  output_topic_ = declare_parameter<std::string>("output_topic", "/lio/odometry");
  world_frame_ = declare_parameter<std::string>("world_frame", "camera_init");
  output_world_frame_ = declare_parameter<std::string>("output_world_frame", "odom");
  imu_frame_ = declare_parameter<std::string>("imu_frame", "body");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  publish_tf_ = declare_parameter<bool>("publish_tf", false);
  publish_world_tf_ = declare_parameter<bool>("publish_world_tf", true);
  covariance_order_ = declare_parameter<std::string>(
      "input_covariance_order", "rotation_position");
  if (input_topic_ == output_topic_ || world_frame_.empty() ||
      output_world_frame_.empty() || imu_frame_.empty() || base_frame_.empty() ||
      world_frame_ == base_frame_ || output_world_frame_ == base_frame_) {
    throw std::invalid_argument("Invalid topic or frame configuration");
  }
  if (covariance_order_ != "rotation_position" &&
      covariance_order_ != "position_rotation") {
    throw std::invalid_argument("Unsupported input_covariance_order");
  }

  // 必须由启动文件传入，不在代码中重复写死 MID360 的 IMU—雷达外参。
  const auto translation = declare_parameter<std::vector<double>>(
      "t_imu_lidar", std::vector<double>{});
  const auto rotation = declare_parameter<std::vector<double>>(
      "R_imu_lidar", std::vector<double>{});
  const auto mount = declare_parameter<std::vector<double>>(
      "t_base_lidar", std::vector<double>{});
  const auto rpy = declare_parameter<std::vector<double>>(
      "rpy_base_lidar", std::vector<double>{});
  const auto floor = declare_parameter<std::vector<double>>(
      "pose_covariance_floor",
      std::vector<double>{0.0025, 0.0025, 0.01, 0.0004, 0.0004, 0.0009});
  validateVector(translation, 3, "t_imu_lidar");
  validateVector(rotation, 9, "R_imu_lidar");
  validateVector(mount, 3, "t_base_lidar");
  validateVector(rpy, 3, "rpy_base_lidar");
  validateVector(floor, 6, "pose_covariance_floor");

  t_imu_lidar_ = Eigen::Vector3d(translation[0], translation[1], translation[2]);
  t_base_lidar_ = Eigen::Vector3d(mount[0], mount[1], mount[2]);
  // YAML 旋转矩阵按行展开，不能按 Eigen 默认的列优先存储直接映射。
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      R_imu_lidar_(row, col) = rotation[3 * row + col];
    }
  }
  if ((R_imu_lidar_.transpose() * R_imu_lidar_ -
       Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
      std::abs(R_imu_lidar_.determinant() - 1.0) > 1e-6) {
    throw std::invalid_argument("R_imu_lidar is not a proper rotation matrix");
  }
  // 与 static_transform_publisher 的 roll/pitch/yaw 定义一致。
  R_base_lidar_ = (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
                      .toRotationMatrix();

  // inverse(T_base_lidar) 的平移为 -R_base_lidar^T*t_base_lidar，
  // 不能只给平移取负，也不能把 imu→lidar 与 lidar→imu 名称混用。
  R_imu_base_ = R_imu_lidar_ * R_base_lidar_.transpose();
  t_imu_base_ = t_imu_lidar_ - R_imu_base_ * t_base_lidar_;
  for (size_t i = 0; i < floor.size(); ++i) {
    if (floor[i] < 0.0) {
      throw std::invalid_argument("pose_covariance_floor must be nonnegative");
    }
    pose_covariance_floor_[i] = floor[i];
  }
}

LidarOdomTransformer::Matrix6d LidarOdomTransformer::convertPoseCovariance(
    const nav_msgs::msg::Odometry &msg, const Eigen::Matrix3d &R_world_imu,
    const Eigen::Matrix3d &R_world_base) const {
  Matrix6d covariance;
  // 本机 FAST-LIO state_ikfom 的状态顺序是 pos,rot，但 publish_odometry
  // 交换了这两块，导致收到的 covariance 为 [rot,pos]；先恢复 [pos,rot]。
  // 若后续已修复 FAST-LIO 发布顺序，将参数改成 position_rotation。
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      const int source_row = covariance_order_ == "rotation_position" ? (row + 3) % 6 : row;
      const int source_col = covariance_order_ == "rotation_position" ? (col + 3) % 6 : col;
      covariance(row, col) = msg.pose.covariance[6 * source_row + source_col];
    }
  }
  if (!covariance.allFinite()) {
    throw std::invalid_argument("Non-finite pose covariance");
  }
  covariance = (0.5 * (covariance + covariance.transpose())).eval();
  Eigen::SelfAdjointEigenSolver<Matrix6d> eig(covariance);
  if (eig.info() != Eigen::Success || eig.eigenvalues().minCoeff() < -1e-6) {
    throw std::invalid_argument("Pose covariance is not positive semidefinite");
  }
  // 修正浮点舍入产生的微小负特征值；首帧全零协方差由输出方差下限补齐。
  covariance = eig.eigenvectors() * eig.eigenvalues().cwiseMax(0.0).asDiagonal() *
               eig.eigenvectors().transpose();

  // IKFoM 的姿态误差是 IMU 局部右扰动，而 ROS 输出角度协方差按 RPY 排列。
  // 局部角度误差先由 R_imu_base^T 转到车体，再转成车体姿态的 RPY 增量。
  const double roll = std::atan2(R_world_base(2, 1), R_world_base(2, 2));
  const double pitch = std::asin(std::clamp(-R_world_base(2, 0), -1.0, 1.0));
  const double cp = std::cos(pitch);
  if (std::abs(cp) < 1e-6) {
    throw std::invalid_argument("RPY covariance singularity near pitch=+-pi/2");
  }
  Eigen::Matrix3d local_angle_to_rpy;
  local_angle_to_rpy << 1.0, std::sin(roll) * std::tan(pitch),
      std::cos(roll) * std::tan(pitch),
      0.0, std::cos(roll), -std::sin(roll),
      0.0, std::sin(roll) / cp, std::cos(roll) / cp;

  Matrix6d jacobian = Matrix6d::Zero();
  jacobian.topLeftCorner<3, 3>().setIdentity();
  jacobian.topRightCorner<3, 3>() = -R_world_imu * skew(t_imu_base_);
  jacobian.bottomRightCorner<3, 3>() =
      local_angle_to_rpy * R_imu_base_.transpose();
  Matrix6d result = jacobian * covariance * jacobian.transpose();
  // 下限是初始噪声估计，不代表已经完成安装误差和传感器噪声标定。
  for (int i = 0; i < 6; ++i) {
    result(i, i) = std::max(result(i, i), pose_covariance_floor_[i]);
  }
  return (0.5 * (result + result.transpose())).eval();
}

void LidarOdomTransformer::initializeOdomFrame(
    const nav_msgs::msg::Odometry &msg, const Eigen::Matrix3d &R_world_base,
    const Eigen::Vector3d &t_world_base) {
  if (world_frame_initialized_) return;
  if (output_world_frame_ != world_frame_) {
    // 首帧车体位置作为原点，只消除初始偏航；不把初始侧倾、俯仰旋入世界轴。
    const double yaw = std::atan2(R_world_base(1, 0), R_world_base(0, 0));
    R_odom_world_ =
        Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    t_odom_world_ = -R_odom_world_ * t_world_base;
    if (world_tf_broadcaster_) {
      // TF 父帧为 odom、子帧为 camera_init；变换把 camera_init 中的点转到 odom。
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = msg.header.stamp;
      transform.header.frame_id = output_world_frame_;
      transform.child_frame_id = world_frame_;
      transform.transform.translation.x = t_odom_world_.x();
      transform.transform.translation.y = t_odom_world_.y();
      transform.transform.translation.z = t_odom_world_.z();
      const Eigen::Quaterniond rotation(R_odom_world_);
      transform.transform.rotation.x = rotation.x();
      transform.transform.rotation.y = rotation.y();
      transform.transform.rotation.z = rotation.z();
      transform.transform.rotation.w = rotation.w();
      world_tf_broadcaster_->sendTransform(transform);
    }
    RCLCPP_INFO(get_logger(), "Initialized fixed %s -> %s from first valid base pose",
                output_world_frame_.c_str(), world_frame_.c_str());
  }
  // 输出帧与输入帧同名时保留原始世界基准，用于独立检查外参。
  world_frame_initialized_ = true;
}

void LidarOdomTransformer::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (msg->header.frame_id != world_frame_ || msg->child_frame_id != imu_frame_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                        "Ignoring unexpected frames %s -> %s",
                        msg->header.frame_id.c_str(), msg->child_frame_id.c_str());
    return;
  }
  try {
    const auto &pose = msg->pose.pose;
    const Eigen::Vector3d t_world_imu(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond q_world_imu(pose.orientation.w, pose.orientation.x,
                                  pose.orientation.y, pose.orientation.z);
    if (!t_world_imu.allFinite() || !q_world_imu.coeffs().allFinite() ||
        q_world_imu.norm() < 1e-12) {
      throw std::invalid_argument("Invalid input position or quaternion");
    }
    q_world_imu.normalize();
    const Eigen::Matrix3d R_world_imu = q_world_imu.toRotationMatrix();
    const Eigen::Matrix3d R_world_base = R_world_imu * R_imu_base_;
    const Eigen::Vector3d t_world_base = t_world_imu + R_world_imu * t_imu_base_;
    // 先检查并换算协方差，无效首帧不得用于建立 odom 基准。
    Matrix6d covariance = convertPoseCovariance(*msg, R_world_imu, R_world_base);
    initializeOdomFrame(*msg, R_world_base, t_world_base);
    const Eigen::Vector3d t_odom_base = R_odom_world_ * t_world_base + t_odom_world_;
    const Eigen::Quaterniond q_odom_base(R_odom_world_ * R_world_base);

    // 左乘纯偏航只改变 RPY 的 yaw 零点，因此角度增量不变；位置增量需旋转。
    // 非对角项也一起换算，不能只交换协方差对角线。
    Matrix6d world_jacobian = Matrix6d::Identity();
    world_jacobian.topLeftCorner<3, 3>() = R_odom_world_;
    covariance = (world_jacobian * covariance * world_jacobian.transpose()).eval();

    nav_msgs::msg::Odometry output;
    output.header = msg->header;  // 保留雷达采样时间，不使用节点当前时间。
    output.header.frame_id = output_world_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose.position.x = t_odom_base.x();
    output.pose.pose.position.y = t_odom_base.y();
    output.pose.pose.position.z = t_odom_base.z();
    output.pose.pose.orientation.w = q_odom_base.w();
    output.pose.pose.orientation.x = q_odom_base.x();
    output.pose.pose.orientation.y = q_odom_base.y();
    output.pose.pose.orientation.z = q_odom_base.z();
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 6; ++col) {
        output.pose.covariance[6 * row + col] = covariance(row, col);
      }
    }

    // 这版 FAST-LIO 未填写 twist：不得把默认零值当作测得的静止速度。
    // 本话题提供位姿，未知速度使用大方差；EKF 只能选择其 pose 字段。
    for (int i = 0; i < 6; ++i) {
      output.twist.covariance[7 * i] = 1e6;
    }
    odom_pub_->publish(output);
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = output.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = output.pose.pose.position.x;
      transform.transform.translation.y = output.pose.pose.position.y;
      transform.transform.translation.z = output.pose.pose.position.z;
      transform.transform.rotation = output.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  } catch (const std::exception &error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                        "Ignoring invalid LIO odometry: %s", error.what());
  }
}

/// 仅启动转换节点，不连接 CAN、雷达或发送底盘运动指令。
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LidarOdomTransformer>());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("lidar_odom_transformer"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

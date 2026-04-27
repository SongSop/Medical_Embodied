#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

class Mid360RotationNode : public rclcpp::Node
{
public:
  Mid360RotationNode()
  : Node("mid360_rotation_node")
  {
    raw_lidar_topic_ = declare_parameter<std::string>("raw_lidar_topic", "/livox/lidar");
    rotated_lidar_topic_ = declare_parameter<std::string>(
      "rotated_lidar_topic", "/livox/lidar_rotated");
    rotated_lidar_points_topic_ = declare_parameter<std::string>(
      "rotated_lidar_points_topic", "/livox/lidar_rotated/points");
    raw_imu_topic_ = declare_parameter<std::string>("raw_imu_topic", "/livox/imu");
    rotated_imu_topic_ = declare_parameter<std::string>(
      "rotated_imu_topic", "/livox/imu_rotated");
    fastlio_lidar_frame_id_ = declare_parameter<std::string>(
      "fastlio_lidar_frame_id", "lidar_3d_frame_fastlio");
    nav_lidar_frame_id_ = declare_parameter<std::string>(
      "nav_lidar_frame_id", "lidar_3d_link");
    rotated_imu_frame_id_ = declare_parameter<std::string>(
      "rotated_imu_frame_id", "imu_link");
    mount_roll_deg_ = declare_parameter<double>("mount_roll_deg", 25.0);
    mount_pitch_deg_ = declare_parameter<double>("mount_pitch_deg", 0.0);
    mount_yaw_deg_ = declare_parameter<double>("mount_yaw_deg", 90.0);
    imu_angular_velocity_bias_ = loadVector3Parameter(
      "imu_angular_velocity_bias", {0.0, 0.0, 0.0});
    imu_angular_velocity_covariance_floor_ = loadVector3Parameter(
      "imu_angular_velocity_covariance_floor", {0.02, 0.02, 0.08});
    imu_linear_acceleration_covariance_floor_ = loadVector3Parameter(
      "imu_linear_acceleration_covariance_floor", {0.5, 0.5, 0.5});

    updateRotationMatrix();

    const auto sensor_qos = rclcpp::SensorDataQoS();
    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    lidar_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      raw_lidar_topic_, sensor_qos,
      std::bind(&Mid360RotationNode::lidarCb, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      raw_imu_topic_, sensor_qos,
      std::bind(&Mid360RotationNode::imuCb, this, std::placeholders::_1));

    rotated_lidar_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(
      rotated_lidar_topic_, reliable_qos);
    rotated_lidar_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      rotated_lidar_points_topic_, reliable_qos);
    rotated_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      rotated_imu_topic_, reliable_qos);

    RCLCPP_INFO(
      get_logger(),
      "MID360 rotation enabled: raw_lidar=%s -> %s (%s) and %s (%s), raw_imu=%s -> %s (%s), "
      "rpy_deg=(%.3f, %.3f, %.3f), imu_gyro_bias=(%.6f, %.6f, %.6f)",
      raw_lidar_topic_.c_str(), rotated_lidar_topic_.c_str(), fastlio_lidar_frame_id_.c_str(),
      rotated_lidar_points_topic_.c_str(), nav_lidar_frame_id_.c_str(),
      raw_imu_topic_.c_str(), rotated_imu_topic_.c_str(), rotated_imu_frame_id_.c_str(),
      mount_roll_deg_, mount_pitch_deg_, mount_yaw_deg_,
      imu_angular_velocity_bias_[0], imu_angular_velocity_bias_[1],
      imu_angular_velocity_bias_[2]);
  }

private:
  std::array<double, 3> loadVector3Parameter(
    const std::string & name,
    const std::array<double, 3> & defaults)
  {
    const auto values = declare_parameter<std::vector<double>>(
      name, std::vector<double>(defaults.begin(), defaults.end()));
    if (values.size() != 3U) {
      RCLCPP_WARN(
        get_logger(), "%s expects 3 values, got %zu. Falling back to defaults.",
        name.c_str(), values.size());
      return defaults;
    }

    return {values[0], values[1], values[2]};
  }

  static double degToRad(double degrees)
  {
    return degrees * M_PI / 180.0;
  }

  void updateRotationMatrix()
  {
    tf2::Quaternion q;
    q.setRPY(
      degToRad(mount_roll_deg_),
      degToRad(mount_pitch_deg_),
      degToRad(mount_yaw_deg_));
    q.normalize();

    tf2::Matrix3x3 rotation_matrix(q);
    for (size_t row = 0; row < 3; ++row) {
      for (size_t col = 0; col < 3; ++col) {
        rotation_[row * 3 + col] = rotation_matrix[static_cast<int>(row)][static_cast<int>(col)];
      }
    }
  }

  std::array<double, 3> rotateVector(double x, double y, double z) const
  {
    return {
      rotation_[0] * x + rotation_[1] * y + rotation_[2] * z,
      rotation_[3] * x + rotation_[4] * y + rotation_[5] * z,
      rotation_[6] * x + rotation_[7] * y + rotation_[8] * z};
  }

  std::array<double, 9> rotateCovariance(const std::array<double, 9> & covariance) const
  {
    if (covariance[0] < 0.0) {
      return covariance;
    }

    std::array<double, 9> temp{};
    std::array<double, 9> result{};

    for (size_t row = 0; row < 3; ++row) {
      for (size_t col = 0; col < 3; ++col) {
        double value = 0.0;
        for (size_t k = 0; k < 3; ++k) {
          value += rotation_[row * 3 + k] * covariance[k * 3 + col];
        }
        temp[row * 3 + col] = value;
      }
    }

    for (size_t row = 0; row < 3; ++row) {
      for (size_t col = 0; col < 3; ++col) {
        double value = 0.0;
        for (size_t k = 0; k < 3; ++k) {
          value += temp[row * 3 + k] * rotation_[col * 3 + k];
        }
        result[row * 3 + col] = value;
      }
    }

    return result;
  }

  std::array<double, 9> applyCovarianceFloor(
    const std::array<double, 9> & covariance,
    const std::array<double, 3> & diagonal_floor) const
  {
    auto result = covariance;
    if (result[0] < 0.0) {
      result.fill(0.0);
    }

    for (size_t i = 0; i < 3; ++i) {
      const size_t index = i * 3 + i;
      if (result[index] <= 0.0 || result[index] < diagonal_floor[i]) {
        result[index] = diagonal_floor[i];
      }
    }

    return result;
  }

  void lidarCb(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    auto rotated_msg = *msg;
    rotated_msg.header.frame_id = fastlio_lidar_frame_id_;
    rotated_msg.point_num = static_cast<uint32_t>(rotated_msg.points.size());

    for (auto & point : rotated_msg.points) {
      const auto rotated = rotateVector(point.x, point.y, point.z);
      point.x = static_cast<float>(rotated[0]);
      point.y = static_cast<float>(rotated[1]);
      point.z = static_cast<float>(rotated[2]);
    }

    rotated_lidar_pub_->publish(rotated_msg);
    publishPointCloud2(rotated_msg);
  }

  void publishPointCloud2(const livox_ros_driver2::msg::CustomMsg & msg)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = msg.header;
    cloud.header.frame_id = nav_lidar_frame_id_;
    cloud.is_bigendian = false;
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(msg.points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");

    for (const auto & point : msg.points) {
      *iter_x = point.x;
      *iter_y = point.y;
      *iter_z = point.z;
      *iter_intensity = static_cast<float>(point.reflectivity);
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
    }

    rotated_lidar_points_pub_->publish(cloud);
  }

  void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto rotated_msg = *msg;
    rotated_msg.header.frame_id = rotated_imu_frame_id_;

    const auto rotated_angular_velocity = rotateVector(
      rotated_msg.angular_velocity.x,
      rotated_msg.angular_velocity.y,
      rotated_msg.angular_velocity.z);
    rotated_msg.angular_velocity.x = rotated_angular_velocity[0] - imu_angular_velocity_bias_[0];
    rotated_msg.angular_velocity.y = rotated_angular_velocity[1] - imu_angular_velocity_bias_[1];
    rotated_msg.angular_velocity.z = rotated_angular_velocity[2] - imu_angular_velocity_bias_[2];

    const auto rotated_linear_acceleration = rotateVector(
      rotated_msg.linear_acceleration.x,
      rotated_msg.linear_acceleration.y,
      rotated_msg.linear_acceleration.z);
    rotated_msg.linear_acceleration.x = rotated_linear_acceleration[0];
    rotated_msg.linear_acceleration.y = rotated_linear_acceleration[1];
    rotated_msg.linear_acceleration.z = rotated_linear_acceleration[2];

    rotated_msg.angular_velocity_covariance = applyCovarianceFloor(
      rotateCovariance(rotated_msg.angular_velocity_covariance),
      imu_angular_velocity_covariance_floor_);
    rotated_msg.linear_acceleration_covariance = applyCovarianceFloor(
      rotateCovariance(rotated_msg.linear_acceleration_covariance),
      imu_linear_acceleration_covariance_floor_);

    rotated_imu_pub_->publish(rotated_msg);
  }

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr rotated_lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rotated_lidar_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr rotated_imu_pub_;

  std::string raw_lidar_topic_;
  std::string rotated_lidar_topic_;
  std::string rotated_lidar_points_topic_;
  std::string raw_imu_topic_;
  std::string rotated_imu_topic_;
  std::string fastlio_lidar_frame_id_;
  std::string nav_lidar_frame_id_;
  std::string rotated_imu_frame_id_;

  double mount_roll_deg_{0.0};
  double mount_pitch_deg_{0.0};
  double mount_yaw_deg_{0.0};
  std::array<double, 3> imu_angular_velocity_bias_{};
  std::array<double, 3> imu_angular_velocity_covariance_floor_{};
  std::array<double, 3> imu_linear_acceleration_covariance_floor_{};
  std::array<double, 9> rotation_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mid360RotationNode>());
  rclcpp::shutdown();
  return 0;
}

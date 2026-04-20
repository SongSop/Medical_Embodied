#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class MapOdomTfPublisher : public rclcpp::Node
{
public:
  MapOdomTfPublisher()
  : Node("map_odom_tf_publisher"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(this)
  {
    source_map_frame_ = this->declare_parameter<std::string>("source_map_frame", "map_fastlio");
    output_map_frame_ = this->declare_parameter<std::string>("output_map_frame", "map");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    fastlio_lidar_frame_ =
      this->declare_parameter<std::string>("fastlio_lidar_frame", "lidar_3d_frame_fastlio");
    global_lidar_frame_ =
      this->declare_parameter<std::string>("global_lidar_frame", "lidar_3d_link");
    publish_rate_ = this->declare_parameter<double>("publish_rate", 30.0);
    tf_lookup_timeout_sec_ = this->declare_parameter<double>("tf_lookup_timeout_sec", 0.05);
    stamp_mode_ = this->declare_parameter<std::string>("stamp_mode", "now");

    const auto timer_period_ms = static_cast<int>(1000.0 / std::max(1.0, publish_rate_));
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(timer_period_ms),
      std::bind(&MapOdomTfPublisher::onTimer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "map->odom 发布器启动(分离TF树模式): source_map=%s, output_map=%s, odom=%s, "
      "fastlio_lidar=%s, global_lidar=%s",
      source_map_frame_.c_str(),
      output_map_frame_.c_str(),
      odom_frame_.c_str(),
      fastlio_lidar_frame_.c_str(),
      global_lidar_frame_.c_str());
  }

private:
  geometry_msgs::msg::TransformStamped lookupTf(
    const std::string & target,
    const std::string & source) const
  {
    return tf_buffer_.lookupTransform(
      target,
      source,
      tf2::TimePointZero,
      tf2::durationFromSec(tf_lookup_timeout_sec_));
  }

  static tf2::Transform toTf2(const geometry_msgs::msg::Transform & msg)
  {
    tf2::Transform tf;
    tf2::fromMsg(msg, tf);
    return tf;
  }

  static geometry_msgs::msg::Transform toMsg(const tf2::Transform & tf)
  {
    return tf2::toMsg(tf);
  }

  void onTimer()
  {
    try {
      // 1) FastLIO 定位输出：map_fastlio -> lidar_3d_frame_fastlio
      auto t_mapf_lfastlio_msg = lookupTf(source_map_frame_, fastlio_lidar_frame_);
      auto t_mapf_lfastlio = toTf2(t_mapf_lfastlio_msg.transform);

      // 2) 主导航树链路：odom -> lidar_3d_link（由 odom->base_link->lidar_3d_link 自动组合）
      auto t_odom_lglobal_msg = lookupTf(odom_frame_, global_lidar_frame_);
      auto t_odom_lglobal = toTf2(t_odom_lglobal_msg.transform);

      // 3) 分离 TF 树模式（按当前项目要求）：
      //    直接将 fastlio_lidar_frame 与 global_lidar_frame 视为同一物理雷达参考帧，
      //    不要求两者之间存在显式 TF。
      //    map_fastlio->odom = (map_fastlio->lidar_3d_frame_fastlio) * inv(odom->lidar_3d_link)
      auto t_mapf_odom = t_mapf_lfastlio * t_odom_lglobal.inverse();

      // 4) 对外发布 map -> odom（默认把 output_map 视作 map_fastlio 对齐坐标）
      geometry_msgs::msg::TransformStamped out;
      if (stamp_mode_ == "source") {
        out.header.stamp = t_mapf_lfastlio_msg.header.stamp;
      } else {
        out.header.stamp = this->get_clock()->now();
      }
      out.header.frame_id = output_map_frame_;
      out.child_frame_id = odom_frame_;
      out.transform = toMsg(t_mapf_odom);

      tf_broadcaster_.sendTransform(out);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "map->odom 未发布，等待 TF 链路完整: %s",
        ex.what());
    }
  }

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string source_map_frame_;
  std::string output_map_frame_;
  std::string odom_frame_;
  std::string fastlio_lidar_frame_;
  std::string global_lidar_frame_;
  std::string stamp_mode_;

  double publish_rate_{30.0};
  double tf_lookup_timeout_sec_{0.05};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapOdomTfPublisher>());
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <limits>
#include <vector>
#include <cmath>

using std::placeholders::_1;

class Pc2ToLaserScan : public rclcpp::Node {
public:
  Pc2ToLaserScan() : Node("pc2_to_laserscan_cpp") {
    // parameters
    this->declare_parameter<std::string>("input_cloud", "/points");
    this->declare_parameter<std::string>("output_laserscan", "/scan");
    this->declare_parameter<double>("angle_min", -M_PI/2.0);
    this->declare_parameter<double>("angle_max", M_PI/2.0);
    this->declare_parameter<double>("angle_increment", 0.0058); // ~0.33 deg
    this->declare_parameter<double>("z_min", -0.1);
    this->declare_parameter<double>("z_max", 1.0);
    this->declare_parameter<double>("range_min", 0.0);
    this->declare_parameter<double>("range_max", 100.0);
    this->declare_parameter<bool>("use_inf", true);
    this->declare_parameter<double>("scan_time", 0.1);

    input_cloud_ = this->get_parameter("input_cloud").as_string();
    output_laserscan_ = this->get_parameter("output_laserscan").as_string();
    angle_min_ = this->get_parameter("angle_min").as_double();
    angle_max_ = this->get_parameter("angle_max").as_double();
    angle_increment_ = this->get_parameter("angle_increment").as_double();
    z_min_ = this->get_parameter("z_min").as_double();
    z_max_ = this->get_parameter("z_max").as_double();
    range_min_ = this->get_parameter("range_min").as_double();
    range_max_ = this->get_parameter("range_max").as_double();
    use_inf_ = this->get_parameter("use_inf").as_bool();
    scan_time_ = this->get_parameter("scan_time").as_double();


    // ---- LOG PARAMETERS ----
    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "  input_cloud: %s", input_cloud_.c_str());
    RCLCPP_INFO(this->get_logger(), "  output_laserscan: %s", output_laserscan_.c_str());
    RCLCPP_INFO(this->get_logger(), "  angle_min: %.3f", angle_min_);
    RCLCPP_INFO(this->get_logger(), "  angle_max: %.3f", angle_max_);
    RCLCPP_INFO(this->get_logger(), "  angle_increment: %.6f", angle_increment_);
    RCLCPP_INFO(this->get_logger(), "  z_min: %.3f", z_min_);
    RCLCPP_INFO(this->get_logger(), "  z_max: %.3f", z_max_);
    RCLCPP_INFO(this->get_logger(), "  range_min: %.3f", range_min_);
    RCLCPP_INFO(this->get_logger(), "  range_max: %.3f", range_max_);
    RCLCPP_INFO(this->get_logger(), "  use_inf: %s", use_inf_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  scan_time: %.3f", scan_time_);

    if (angle_increment_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "angle_increment <= 0, setting to default 0.0058");
      angle_increment_ = 0.0058;
    }

    size_t num_bins = (size_t)std::ceil((angle_max_ - angle_min_) / angle_increment_) + 1;
    num_ranges_ = num_bins;

    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_laserscan_, 10);
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_, 10, std::bind(&Pc2ToLaserScan::cloud_cb, this, _1));
  }

private:
  void cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // initialize ranges with inf or max
    std::vector<float> ranges(num_ranges_, std::numeric_limits<float>::infinity());
    std::vector<bool> seen(num_ranges_, false);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      float x = *iter_x;
      float y = *iter_y;
      float z = *iter_z;

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

      if (z < z_min_ || z > z_max_) continue;

      float range = std::hypot(x, y);
      if (range < range_min_ || range > range_max_) continue;

      float angle = std::atan2(y, x);
      if (angle < angle_min_ || angle > angle_max_) continue;

      int idx = (int)std::floor((angle - angle_min_) / angle_increment_);
      if (idx < 0 || (size_t)idx >= num_ranges_) continue;

      if (!seen[idx] || range < ranges[idx]) {
        ranges[idx] = range;
        seen[idx] = true;
      }
    }

    // replace infinities with range_max or inf depending on use_inf_
    for (size_t i=0;i<ranges.size();++i) {
      if (!seen[i]) {
        if (!use_inf_) ranges[i] = std::numeric_limits<float>::infinity(); // keep inf
        else ranges[i] = std::numeric_limits<float>::infinity();
      }
    }

    auto scan = sensor_msgs::msg::LaserScan();
    scan.header = msg->header;
    scan.header.stamp = this->now();
    scan.angle_min = angle_min_;
    scan.angle_max = angle_max_;
    scan.angle_increment = angle_increment_;
    scan.time_increment = 0.0;
    scan.scan_time = scan_time_;
    scan.range_min = range_min_;
    scan.range_max = range_max_;
    scan.ranges.resize(ranges.size());
    for (size_t i=0;i<ranges.size();++i) {
      scan.ranges[i] = ranges[i];
    }

    pub_->publish(scan);
  }

  // params
  std::string input_cloud_, output_laserscan_;
  double angle_min_, angle_max_, angle_increment_;
  double z_min_, z_max_, range_min_, range_max_, scan_time_;
  bool use_inf_;
  size_t num_ranges_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pc2ToLaserScan>());
  rclcpp::shutdown();
  return 0;
}

#include "lidar_angle_filter/angle_filter.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace lidar_angle_filter
{

struct CoordinateField
{
  std::uint32_t offset;
  std::uint8_t datatype;
};

class LidarAngleFilterNode : public rclcpp::Node
{
public:
  LidarAngleFilterNode()
  : Node("lidar_angle_filter")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ = declare_parameter<std::string>("output_topic", "/livox/lidar_filtered");
    front_center_deg_ = declare_parameter<double>("front_center_deg", 0.0);
    front_sector_deg_ = declare_parameter<double>("front_sector_deg", 30.0);
    back_sector_deg_ = declare_parameter<double>("back_sector_deg", 30.0);

    validate_parameters();

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarAngleFilterNode::filter_cloud, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Filtering %.1f deg at the front (center %.1f deg) and %.1f deg at the back: %s -> %s",
      front_sector_deg_, front_center_deg_, back_sector_deg_, input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void validate_parameters() const
  {
    if (input_topic_ == output_topic_) {
      throw std::invalid_argument("input_topic and output_topic must be different");
    }
    if (!std::isfinite(front_center_deg_)) {
      throw std::invalid_argument("front_center_deg must be finite");
    }
    if (!valid_sector_width(front_sector_deg_) || !valid_sector_width(back_sector_deg_)) {
      throw std::invalid_argument("front_sector_deg and back_sector_deg must be between 0 and 180");
    }
  }

  static bool valid_sector_width(const double width)
  {
    return std::isfinite(width) && width >= 0.0 && width <= 180.0;
  }

  static CoordinateField find_coordinate_field(
    const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
  {
    const auto field = std::find_if(
      cloud.fields.begin(), cloud.fields.end(),
      [&name](const sensor_msgs::msg::PointField & candidate) {return candidate.name == name;});

    if (field == cloud.fields.end()) {
      throw std::runtime_error("PointCloud2 is missing the '" + name + "' field");
    }
    if (field->datatype != sensor_msgs::msg::PointField::FLOAT32 &&
      field->datatype != sensor_msgs::msg::PointField::FLOAT64)
    {
      throw std::runtime_error("PointCloud2 field '" + name + "' must be FLOAT32 or FLOAT64");
    }

    const std::size_t field_size =
      field->datatype == sensor_msgs::msg::PointField::FLOAT32 ? sizeof(float) : sizeof(double);
    if (field->offset + field_size > cloud.point_step) {
      throw std::runtime_error("PointCloud2 field '" + name + "' extends beyond point_step");
    }
    return {field->offset, field->datatype};
  }

  template<typename T>
  static T byte_swap(T value)
  {
    std::uint8_t source[sizeof(T)];
    std::uint8_t destination[sizeof(T)];
    std::memcpy(source, &value, sizeof(T));
    std::reverse_copy(source, source + sizeof(T), destination);
    std::memcpy(&value, destination, sizeof(T));
    return value;
  }

  static bool host_is_big_endian()
  {
    const std::uint16_t value = 0x0102;
    return *reinterpret_cast<const std::uint8_t *>(&value) == 0x01;
  }

  static double read_coordinate(
    const std::uint8_t * point, const CoordinateField & field, const bool cloud_is_big_endian)
  {
    if (field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      float value;
      std::memcpy(&value, point + field.offset, sizeof(value));
      if (cloud_is_big_endian != host_is_big_endian()) {
        value = byte_swap(value);
      }
      return value;
    }

    double value;
    std::memcpy(&value, point + field.offset, sizeof(value));
    if (cloud_is_big_endian != host_is_big_endian()) {
      value = byte_swap(value);
    }
    return value;
  }

  void filter_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    try {
      const CoordinateField x_field = find_coordinate_field(*cloud, "x");
      const CoordinateField y_field = find_coordinate_field(*cloud, "y");

      auto filtered = std::make_unique<sensor_msgs::msg::PointCloud2>();
      filtered->header = cloud->header;
      filtered->height = 1;
      filtered->fields = cloud->fields;
      filtered->is_bigendian = cloud->is_bigendian;
      filtered->point_step = cloud->point_step;
      filtered->is_dense = cloud->is_dense;
      filtered->data.reserve(cloud->data.size());

      std::size_t kept_points = 0;
      std::size_t masked_points = 0;
      for (std::uint32_t row = 0; row < cloud->height; ++row) {
        const std::size_t row_offset = static_cast<std::size_t>(row) * cloud->row_step;
        for (std::uint32_t column = 0; column < cloud->width; ++column) {
          const std::size_t offset = row_offset + static_cast<std::size_t>(column) * cloud->point_step;
          if (offset + cloud->point_step > cloud->data.size()) {
            throw std::runtime_error("PointCloud2 data is shorter than its dimensions declare");
          }

          const std::uint8_t * point = cloud->data.data() + offset;
          const double x = read_coordinate(point, x_field, cloud->is_bigendian);
          const double y = read_coordinate(point, y_field, cloud->is_bigendian);
          if (point_is_masked(
              x, y, front_center_deg_, front_sector_deg_, back_sector_deg_))
          {
            ++masked_points;
            continue;
          }

          filtered->data.insert(filtered->data.end(), point, point + cloud->point_step);
          ++kept_points;
        }
      }

      filtered->width = static_cast<std::uint32_t>(kept_points);
      filtered->row_step = filtered->width * filtered->point_step;
      publisher_->publish(std::move(filtered));

      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 5000, "Kept %zu points; masked %zu points",
        kept_points, masked_points);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  double front_center_deg_;
  double front_sector_deg_;
  double back_sector_deg_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace lidar_angle_filter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<lidar_angle_filter::LidarAngleFilterNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("lidar_angle_filter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

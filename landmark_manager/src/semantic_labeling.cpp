#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

struct PointHash {
  size_t operator()(const geometry_msgs::msg::Point& point) const {
    return std::hash<double>()(point.x) ^
           std::hash<double>()(point.y) ^
           std::hash<double>()(point.z);
  }
};

struct PointEqual {
  bool operator()(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) const {
    const double threshold = 0.1; // Threshold for considering points as equal
    return std::abs(p1.x - p2.x) < threshold &&
           std::abs(p1.y - p2.y) < threshold &&
           std::abs(p1.z - p2.z) < threshold;
  }
};

class SemanticLabeling : public rclcpp::Node
{
public:
  SemanticLabeling()
  : Node("semantic_labeling")
  {
    // Initializes variables for publishers, subscribers, timer, tf buffer, and tf listener
    rate_ = 200;
    door_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/doors", 10);
    table_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/tables", 10);
    door_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "door", 10, std::bind(
        &SemanticLabeling::door_callback, this,
        std::placeholders::_1));
    table_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "table", 10, std::bind(
        &SemanticLabeling::table_callback, this,
        std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / rate_),
      std::bind(&SemanticLabeling::timer_callback, this));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    door_marker_id_ = 0;
    table_marker_id_ = 0;
    door_x_ = 0.0;
    door_y_ = 0.0;
    door_z_ = 0.0;
    table_x_ = 0.0;
    table_y_ = 0.0;
    table_z_ = 0.0;
    new_door_point_received_ = false;
    new_table_point_received_ = false;

    // Initialize unique sets for doors and tables to prevent duplication
    unique_door_points.clear();
    unique_table_points.clear();
  }

private:
  void timer_callback()
  {
    // Check if a new door point has been received
    if (new_door_point_received_)
    {
      door_marker_pub_->publish(door_marker_array_);

      // Reset the flag
      new_door_point_received_ = false;
    }

    // Check if a new table point has been received
    if (new_table_point_received_)
    {
      table_marker_pub_->publish(table_marker_array_);

      // Reset the flag
      new_table_point_received_ = false;
    }
  }

//   void door_callback(const geometry_msgs::msg::Point & msg)
//   {
//     // auto transformed_point = transform_point(*msg, "map");

//     door_x_ = msg.x;
//     door_y_ = msg.y;
//     door_z_ = msg.z;

//     door_marker_array_.markers.clear();

//     visualization_msgs::msg::Marker door_marker;
//     // door_marker.header.frame_id = transformed_point.header.frame_id;
//     door_marker.header.frame_id = "base_camera";
//     door_marker.header.stamp = get_clock()->now();
//     door_marker.id = door_marker_id_++;
//     door_marker.type = visualization_msgs::msg::Marker::SPHERE;
//     door_marker.action = visualization_msgs::msg::Marker::ADD;
//     // door_marker.pose.position.x = transformed_point.point.x;
//     // door_marker.pose.position.y = transformed_point.point.y;
//     // door_marker.pose.position.z = transformed_point.point.z;
//     door_marker.pose.position.x = door_x_;
//     door_marker.pose.position.y = door_y_;
//     door_marker.pose.position.z = door_z_;
//     door_marker.pose.orientation.w = 1.0;
//     door_marker.scale.x = 0.5;
//     door_marker.scale.y = 0.5;
//     door_marker.scale.z = 0.5;
//     door_marker.color.b = 1.0;
//     door_marker.color.a = 1.0;

//     door_marker_array_.markers.push_back(door_marker);

//     // Set the flag to indicate a new door point has been received
//     new_door_point_received_ = true;
//   }

//   void table_callback(const geometry_msgs::msg::Point & msg)
//   {
//     // auto transformed_point = transform_point(*msg, "map");

//     table_x_ = msg.x;
//     table_y_ = msg.y;
//     table_z_ = msg.z;

//     table_marker_array_.markers.clear();

//     visualization_msgs::msg::Marker table_marker;
//     // table_marker.header.frame_id = transformed_point.header.frame_id;
//     table_marker.header.frame_id = "base_camera";
//     table_marker.header.stamp = get_clock()->now();
//     table_marker.id = table_marker_id_++;
//     table_marker.type = visualization_msgs::msg::Marker::SPHERE;
//     table_marker.action = visualization_msgs::msg::Marker::ADD;
//     // table_marker.pose.position.x = transformed_point.point.x;
//     // table_marker.pose.position.y = transformed_point.point.y;
//     // table_marker.pose.position.z = transformed_point.point.z;
//     table_marker.pose.position.x = table_x_;
//     table_marker.pose.position.y = table_y_;
//     table_marker.pose.position.z = table_z_;
//     table_marker.pose.orientation.w = 1.0;
//     table_marker.scale.x = 0.5;
//     table_marker.scale.y = 0.5;
//     table_marker.scale.z = 0.5;
//     table_marker.color.b = 1.0;
//     table_marker.color.a = 1.0;

//     table_marker_array_.markers.push_back(table_marker);

//     // Set the flag to indicate a new table point has been received
//     new_table_point_received_ = true;
//   }

  void door_callback(const geometry_msgs::msg::Point & msg) {
    if (unique_door_points.find(msg) == unique_door_points.end()) {
      unique_door_points.insert(msg);
      auto transformed_point = transform_point(msg, "map");
      create_door_marker(transformed_point.point);
      new_door_point_received_ = true;
    }
  }

  void table_callback(const geometry_msgs::msg::Point & msg) {
    if (unique_table_points.find(msg) == unique_table_points.end()) {
      unique_table_points.insert(msg);
      auto transformed_point = transform_point(msg, "map");
      create_table_marker(transformed_point.point);
      new_table_point_received_ = true;
    }
  }

  void create_door_marker(const geometry_msgs::msg::Point & point) {
    door_marker_array_.markers.clear();

    visualization_msgs::msg::Marker door_marker;
    door_marker.header.frame_id = "map";
    door_marker.header.stamp = get_clock()->now();
    door_marker.id = door_marker_id_++;
    door_marker.type = visualization_msgs::msg::Marker::SPHERE;
    door_marker.action = visualization_msgs::msg::Marker::ADD;
    door_marker.pose.position = point;
    door_marker.pose.orientation.w = 1.0;
    door_marker.scale.x = 0.5;
    door_marker.scale.y = 0.5;
    door_marker.scale.z = 0.5;
    door_marker.color.b = 1.0;
    door_marker.color.a = 1.0;

    door_marker_array_.markers.push_back(door_marker);
  }

  void create_table_marker(const geometry_msgs::msg::Point & point) {
    table_marker_array_.markers.clear();

    visualization_msgs::msg::Marker table_marker;
    table_marker.header.frame_id = "map";
    table_marker.header.stamp = get_clock()->now();
    table_marker.id = table_marker_id_++;
    table_marker.type = visualization_msgs::msg::Marker::SPHERE;
    table_marker.action = visualization_msgs::msg::Marker::ADD;
    table_marker.pose.position = point;
    table_marker.pose.orientation.w = 1.0;
    table_marker.scale.x = 0.5;
    table_marker.scale.y = 0.5;
    table_marker.scale.z = 0.5;
    table_marker.color.b = 1.0;
    table_marker.color.a = 1.0;

    table_marker_array_.markers.push_back(table_marker);
  }

  geometry_msgs::msg::PointStamped transform_point(const geometry_msgs::msg::Point &point_in,
    const std::string &target_frame)
  {
    geometry_msgs::msg::PointStamped point_stamped_in, point_stamped_out;
    point_stamped_in.header.frame_id = "camera_link";
    point_stamped_in.header.stamp = get_clock()->now();
    point_stamped_in.point = point_in;

    try
    {
      point_stamped_out = tf_buffer_->transform(point_stamped_in, target_frame,
                                               tf2::durationFromSec(0.1));
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN(rclcpp::get_logger("SemanticLabeling"),
                  "Transformation exception: %s", ex.what());
    }
    return point_stamped_out;
  }

  // Declare private variables
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr door_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr table_marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr door_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr table_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  int rate_;
  visualization_msgs::msg::MarkerArray door_marker_array_;
  visualization_msgs::msg::MarkerArray table_marker_array_;
  int door_marker_id_;
  int table_marker_id_;
  double door_x_;
  double door_y_;
  double door_z_;
  double table_x_;
  double table_y_;
  double table_z_;
  bool new_door_point_received_;
  bool new_table_point_received_;

  // Unique sets to track unique points
  std::unordered_set<geometry_msgs::msg::Point, PointHash, PointEqual> unique_door_points;
  std::unordered_set<geometry_msgs::msg::Point, PointHash, PointEqual> unique_table_points;
};

/// \brief The main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SemanticLabeling>());
  rclcpp::shutdown();
  return 0;
}

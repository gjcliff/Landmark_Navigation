#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

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
    const double threshold = 0.5;
    auto hash_x = std::hash<double>()(std::floor(point.x / threshold) * threshold);
    auto hash_y = std::hash<double>()(std::floor(point.y / threshold) * threshold);
    auto hash_z = std::hash<double>()(std::floor(point.z / threshold) * threshold);
    return hash_x ^ (hash_y << 1) ^ (hash_z << 2); // Use bitwise shifts to combine hashes
  }
};

struct PointEqual {
  bool operator()(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) const {
    const double threshold = 0.5; // Threshold for considering points as equal
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
    rate_ = 20;
    door_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/doors", 10);
    table_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/tables", 10);
    door_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "door", 10, std::bind(
        &SemanticLabeling::door_callback, this,
        std::placeholders::_1));
    table_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
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
    new_door_point_received_ = false;
    new_table_point_received_ = false;

    // Initialize unique maps for doors and tables to prevent duplication
    unique_door_points_.clear();
    unique_table_points_.clear();
  }

private:
  void timer_callback()
  {
    print_unique_door_points();

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

  // void door_callback(const geometry_msgs::msg::PointStamped & msg) {
  //   auto transformed_point = transform_point(msg, "map");
  //   if (unique_door_points.find(transformed_point.point) == unique_door_points.end()) {
  //     unique_door_points.insert(transformed_point.point);
  //     create_door_marker(transformed_point.point);
  //     new_door_point_received_ = true;
  //   }
  // }

  // void table_callback(const geometry_msgs::msg::PointStamped & msg) {
  //   auto transformed_point = transform_point(msg, "map");
  //   if (unique_table_points.find(transformed_point.point) == unique_table_points.end()) {
  //     unique_table_points.insert(transformed_point.point);
  //     create_table_marker(transformed_point.point);
  //     new_table_point_received_ = true;
  //   }
  // }

  void door_callback(const geometry_msgs::msg::PointStamped & msg) {
    auto transformed_point = transform_point(msg, "map");
    auto it = unique_door_points_.find(transformed_point.point);
    if (it == unique_door_points_.end()) {
      // Insert the point with a new marker ID if it's not found in the set
      int new_id = door_marker_id_++;
      unique_door_points_[transformed_point.point] = new_id;
      create_door_marker(transformed_point.point, new_id);
      new_door_point_received_ = true;
    } else {
      // If the point is already present, use its existing marker ID
      create_door_marker(transformed_point.point, it->second);
      new_door_point_received_ = true;
    }
  }

  void table_callback(const geometry_msgs::msg::PointStamped & msg) {
    auto transformed_point = transform_point(msg, "map");
    auto it = unique_table_points_.find(transformed_point.point);
    if (it == unique_table_points_.end()) {
      // Insert the point with a new marker ID if it's not found in the set
      int new_id = table_marker_id_++;
      unique_table_points_[transformed_point.point] = new_id;
      create_table_marker(transformed_point.point, new_id);
      new_table_point_received_ = true;
    } else {
      // If the point is already present, use its existing marker ID
      create_table_marker(transformed_point.point, it->second);
      new_table_point_received_ = true;
    }
  }

  void create_door_marker(const geometry_msgs::msg::Point & point, int marker_id) {
    // door_marker_array_.markers.clear();

    visualization_msgs::msg::Marker door_marker;
    door_marker.header.frame_id = "map";
    door_marker.header.stamp = get_clock()->now();
    door_marker.id = marker_id;
    door_marker.type = visualization_msgs::msg::Marker::SPHERE;
    door_marker.action = visualization_msgs::msg::Marker::ADD;
    door_marker.pose.position = point;
    door_marker.pose.orientation.w = 1.0;
    door_marker.scale.x = 0.5;
    door_marker.scale.y = 0.5;
    door_marker.scale.z = 0.5;
    door_marker.color.b = 1.0;
    door_marker.color.a = 1.0;
    // door_marker.lifetime.sec = 1;

    // Update existing marker or add new marker
    auto it = std::find_if(door_marker_array_.markers.begin(), door_marker_array_.markers.end(),
                          [marker_id](const auto& marker) { return marker.id == marker_id; });
    if (it != door_marker_array_.markers.end()) {
      // Update the existing marker
      *it = door_marker;
    } else {
      // Add new marker to the array
      door_marker_array_.markers.push_back(door_marker);
    }
  }

  void create_table_marker(const geometry_msgs::msg::Point & point, int marker_id) {
    // table_marker_array_.markers.clear();

    visualization_msgs::msg::Marker table_marker;
    table_marker.header.frame_id = "map";
    table_marker.header.stamp = get_clock()->now();
    table_marker.id = marker_id;
    table_marker.type = visualization_msgs::msg::Marker::SPHERE;
    table_marker.action = visualization_msgs::msg::Marker::ADD;
    table_marker.pose.position = point;
    table_marker.pose.orientation.w = 1.0;
    table_marker.scale.x = 0.5;
    table_marker.scale.y = 0.5;
    table_marker.scale.z = 0.5;
    table_marker.color.b = 1.0;
    table_marker.color.a = 1.0;
    // table_marker.lifetime.sec = 1;

    // Update existing marker or add new marker
    auto it = std::find_if(table_marker_array_.markers.begin(), table_marker_array_.markers.end(),
                          [marker_id](const auto& marker) { return marker.id == marker_id; });
    if (it != table_marker_array_.markers.end()) {
      // Update the existing marker
      *it = table_marker;
    } else {
      // Add new marker to the array
      table_marker_array_.markers.push_back(table_marker);
    }
  }

  geometry_msgs::msg::PointStamped transform_point(
  const geometry_msgs::msg::PointStamped &point_stamped_in, const std::string &target_frame)
  {
    geometry_msgs::msg::PointStamped point_stamped_out;

    try
    {
      if (!tf_buffer_->canTransform(target_frame, point_stamped_in.header.frame_id,
                                    point_stamped_in.header.stamp, tf2::durationFromSec(1.0)))
      {
        RCLCPP_INFO(rclcpp::get_logger("SemanticLabeling"), "Waiting for transform...");
      }
      
      point_stamped_out = tf_buffer_->transform(point_stamped_in, target_frame,
                                                tf2::durationFromSec(0.1));

      // RCLCPP_INFO(rclcpp::get_logger("SemanticLabeling"),
      //             "Transformed [%f, %f, %f] from frame '%s' to frame '%s'.",
      //             point_stamped_in.point.x, point_stamped_in.point.y, point_stamped_in.point.z,
      //             point_stamped_in.header.frame_id.c_str(), target_frame.c_str());
      
      // RCLCPP_INFO(rclcpp::get_logger("SemanticLabeling"),
      //             "Map coordinates [%f, %f, %f] after transformation from frame '%s' to frame '%s'.",
      //             point_stamped_out.point.x, point_stamped_out.point.y, point_stamped_out.point.z,
      //             point_stamped_in.header.frame_id.c_str(), target_frame.c_str());
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN(rclcpp::get_logger("SemanticLabeling"),
                  "Transformation exception: %s", ex.what());
    }
    return point_stamped_out;
  }

  void print_unique_door_points() {
    for (const auto& pair : unique_door_points_) {
      const auto& point = pair.first;
      int marker_id = pair.second;
      RCLCPP_INFO(rclcpp::get_logger("SemanticLabeling"), "Door Point - x: %f, y: %f, z: %f, Marker ID: %d", 
                  point.x, point.y, point.z, marker_id);
    }
  }

  // Declare private variables
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr door_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr table_marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr door_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr table_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  int rate_;
  visualization_msgs::msg::MarkerArray door_marker_array_;
  visualization_msgs::msg::MarkerArray table_marker_array_;
  int door_marker_id_;
  int table_marker_id_;
  bool new_door_point_received_;
  bool new_table_point_received_;

  // // Unique sets to track unique points
  // std::unordered_set<geometry_msgs::msg::Point, PointHash, PointEqual> unique_door_points_;
  // std::unordered_set<geometry_msgs::msg::Point, PointHash, PointEqual> unique_table_points_;

  // Maps to track unique points to marker IDs
  std::unordered_map<geometry_msgs::msg::Point, int, PointHash, PointEqual> unique_door_points_;
  std::unordered_map<geometry_msgs::msg::Point, int, PointHash, PointEqual> unique_table_points_;
};

/// \brief The main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SemanticLabeling>());
  rclcpp::shutdown();
  return 0;
}

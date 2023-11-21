/// \file
/// \brief This node performs semantic labeling by publishing markers for detected doors and tables
///
/// PARAMETERS:
///     rate (int): a frequency used for the timer
///     threshold (double): the threshold for considering points as equal
/// PUBLISHES:
///     /semantic_labeling/doors (visualization_msgs::msg::MarkerArray): sphere markers that
///                                                                      represent doors as well as
///                                                                      text markers
///     /semantic_labeling/tables (visualization_msgs::msg::MarkerArray): sphere markers that
///                                                                       represent tables as well
///                                                                       as text markers
///     /semantic_labeling/semantic_doors (landmark_manager::msg::SemanticPoint): coordinates of a
///                                                                               detected door in
///                                                                               the map frame and
///                                                                               the associated
///                                                                               marker id
///     /semantic_labeling/semantic_tables (landmark_manager::msg::SemanticPoint): coordinates of a
///                                                                                detected table
///                                                                                in the map frame
///                                                                                and the
///                                                                                associated
///                                                                                marker id
/// SUBSCRIBES:
///     /door (geometry_msgs::msg::PointStamped): coordinates of a detected door
///     /table (geometry_msgs::msg::PointStamped): coordinates of a detected table

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "landmark_manager/msg/semantic_point.hpp"

using namespace std::chrono_literals;

/// \brief Uses markers to label doors and tables
class SemanticLabeling : public rclcpp::Node
{
public:
  SemanticLabeling()
  : Node("semantic_labeling")
  {
    // Initializes variables for the parameters, publishers, subscribers, timer, tf buffer, and
    // tf listener
    declare_parameter("rate", 200);
    declare_parameter("threshold", 2.5);
    rate_ = get_parameter("rate").get_parameter_value().get<int>();
    threshold_ = get_parameter("threshold").get_parameter_value().get<double>();

    door_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/doors", 10);
    table_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/tables", 10);
    semantic_door_pub_ = create_publisher<landmark_manager::msg::SemanticPoint>(
      "~/semantic_doors",
      10);
    semantic_table_pub_ = create_publisher<landmark_manager::msg::SemanticPoint>(
      "~/semantic_tables", 10);
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

    // Initializes vectors for unique doors and tables to prevent duplication
    unique_door_points_.clear();
    unique_table_points_.clear();
  }

private:
  /// \brief Callback function for the timer. Checks if new door or table points have been received
  /// and publishes corresponding markers.
  ///
  /// \param none
  /// \return none
  void timer_callback()
  {
    // Check if a new door point has been received
    if (new_door_point_received_) {
      door_marker_pub_->publish(door_marker_array_);

      // Reset the flag
      new_door_point_received_ = false;
    }

    // Check if a new table point has been received
    if (new_table_point_received_) {
      table_marker_pub_->publish(table_marker_array_);

      // Reset the flag
      new_table_point_received_ = false;
    }
  }

  /// \brief Callback function for the subscriber that subscribes to /door. Transforms received
  /// door point to the map frame, checks for uniqueness, and updates markers.
  ///
  /// \param msg - PointStamped object
  /// \return none
  void door_callback(const geometry_msgs::msg::PointStamped & msg)
  {
    // Transform the received point to the map frame
    auto transformed_optional = transform_point(msg, "map");

    // Check if the transformation was successful
    if (transformed_optional) {
      auto transformed_point = *transformed_optional;
      int new_id;

      // Search for the point in the list of unique door points
      auto it = std::find_if(
        unique_door_points_.begin(), unique_door_points_.end(),
        [this, &transformed_point](const auto & pair) {
          // Check if the current point matches the transformed point
          return this->equal_points(pair.first, transformed_point.point);
        });

      // If the point is not already in the list of unique door points
      if (it == unique_door_points_.end()) {
        // Generate a new id
        new_id = door_marker_id_++;

        // Add the new point and its id to the list of unique door points
        unique_door_points_.emplace_back(transformed_point.point, new_id);

        // Create a new door marker
        create_door_marker(transformed_point.point, new_id, transformed_point.header.stamp);
        new_door_point_received_ = true;
      } else {
        // Update the existing door marker
        new_id = it->second;
        create_door_marker(it->first, it->second, transformed_point.header.stamp);
        new_door_point_received_ = true;
      }

      // Publish the point in the map frame and marker id
      landmark_manager::msg::SemanticPoint semantic_msg;
      semantic_msg.point = transformed_point.point;
      semantic_msg.marker_id = new_id;
      semantic_door_pub_->publish(semantic_msg);
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("SemanticLabeling"),
        "Transformation failed. Point not processed.");
    }
  }

  /// \brief Callback function for the subscriber that subscribes to /table. Transforms received
  /// table point to the map frame, checks for uniqueness, and updates markers.
  ///
  /// \param msg - PointStamped object
  /// \return none
  void table_callback(const geometry_msgs::msg::PointStamped & msg)
  {
    // Transform the received point to the map frame
    auto transformed_optional = transform_point(msg, "map");
    int new_id;

    // Check if the transformation was successful
    if (transformed_optional) {
      auto transformed_point = *transformed_optional;

      // Search for the point in the list of unique table points
      auto it = std::find_if(
        unique_table_points_.begin(), unique_table_points_.end(),
        [this, &transformed_point](const auto & pair) {
          // Check if the current point matches the transformed point
          return this->equal_points(pair.first, transformed_point.point);
        });

      // If the point is not already in the list of unique table points
      if (it == unique_table_points_.end()) {
        // Generate a new id
        new_id = table_marker_id_++;

        // Add the new point and its id to the list of unique table points
        unique_table_points_.emplace_back(transformed_point.point, new_id);

        // Create a new table marker
        create_table_marker(transformed_point.point, new_id, transformed_point.header.stamp);
        new_table_point_received_ = true;
      } else {
        // Update the existing table marker
        new_id = it->second;
        create_table_marker(it->first, it->second, transformed_point.header.stamp);
        new_table_point_received_ = true;
      }

      // Publish the point in the map frame and marker id
      landmark_manager::msg::SemanticPoint semantic_msg;
      semantic_msg.point = transformed_point.point;
      semantic_msg.marker_id = new_id;
      semantic_door_pub_->publish(semantic_msg);
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("SemanticLabeling"),
        "Transformation failed. Point not processed.");
    }
  }

  /// \brief Creates or updates a marker for a door. Creates a text marker for labeling.
  ///
  /// \param point - the point to create a marker at
  /// \param marker_id - the id for the marker
  /// \param stamp - the time stamp for the marker
  /// \return none
  void create_door_marker(
    const geometry_msgs::msg::Point & point, int marker_id,
    builtin_interfaces::msg::Time stamp)
  {
    visualization_msgs::msg::Marker door_marker;
    door_marker.header.frame_id = "map";
    door_marker.header.stamp = stamp;
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

    // Update existing marker or add new marker
    auto it = std::find_if(
      door_marker_array_.markers.begin(), door_marker_array_.markers.end(),
      [marker_id](const auto & marker) {return marker.id == marker_id;});
    if (it != door_marker_array_.markers.end()) {
      // Update the existing marker
      *it = door_marker;
    } else {
      // Add new marker to the array
      door_marker_array_.markers.push_back(door_marker);
    }

    // Check if the door marker has been labeled before
    if (labeled_door_marker_ids_.find(marker_id) == labeled_door_marker_ids_.end()) {
      // Create a new text marker
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = "map";
      text_marker.header.stamp = stamp;
      text_marker.id = marker_id + 1000; // Offset to avoid id clashes
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.pose.position = point;
      text_marker.pose.position.x += 0.5;
      text_marker.pose.orientation.w = 1.0;
      text_marker.scale.z = 0.5;
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      text_marker.color.a = 1.0;
      std::ostringstream label_stream;
      label_stream << "Door" << marker_id;
      text_marker.text = label_stream.str();

      // Add text marker to the array
      door_marker_array_.markers.push_back(text_marker);

      // Add door marker id to keep track of labels
      labeled_door_marker_ids_.insert(marker_id);
    }
  }

  /// \brief Creates or updates a marker for a table. Creates a text marker for labeling.
  ///
  /// \param point - the point to create a marker at
  /// \param marker_id - the id for the marker
  /// \param stamp - the time stamp for the marker
  /// \return none
  void create_table_marker(
    const geometry_msgs::msg::Point & point, int marker_id,
    builtin_interfaces::msg::Time stamp)
  {
    visualization_msgs::msg::Marker table_marker;
    table_marker.header.frame_id = "map";
    table_marker.header.stamp = stamp;
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

    // Update existing marker or add new marker
    auto it = std::find_if(
      table_marker_array_.markers.begin(), table_marker_array_.markers.end(),
      [marker_id](const auto & marker) {return marker.id == marker_id;});
    if (it != table_marker_array_.markers.end()) {
      // Update the existing marker
      *it = table_marker;
    } else {
      // Add new marker to the array
      table_marker_array_.markers.push_back(table_marker);
    }

    // Check if the table marker has been labeled before
    if (labeled_table_marker_ids_.find(marker_id) == labeled_table_marker_ids_.end()) {
      // Create a new text marker
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = "map";
      text_marker.header.stamp = stamp;
      text_marker.id = marker_id + 1000; // Offset to avoid id clashes
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.pose.position = point;
      text_marker.pose.position.x += 0.5;
      text_marker.pose.orientation.w = 1.0;
      text_marker.scale.z = 0.5;
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      text_marker.color.a = 1.0;
      std::ostringstream label_stream;
      label_stream << "Table" << marker_id;
      text_marker.text = label_stream.str();

      // Add text marker to the array
      table_marker_array_.markers.push_back(text_marker);

      // Add table marker id to keep track of labels
      labeled_table_marker_ids_.insert(marker_id);
    }
  }

  /// \brief Transforms a point to a given target frame
  ///
  /// \param point_stamped_in - the point to be transformed
  /// \param target_frame - the target frame to transform the point to
  /// \return the transformed point; empty if transformation fails
  std::optional<geometry_msgs::msg::PointStamped> transform_point(
    const geometry_msgs::msg::PointStamped & point_stamped_in, const std::string & target_frame)
  {
    try {
      if (!tf_buffer_->canTransform(
          target_frame, point_stamped_in.header.frame_id,
          point_stamped_in.header.stamp, tf2::durationFromSec(1.0)))
      {
        RCLCPP_INFO(rclcpp::get_logger("SemanticLabeling"), "Waiting for transform...");
      }

      geometry_msgs::msg::PointStamped point_stamped_out =
        tf_buffer_->transform(point_stamped_in, target_frame, tf2::durationFromSec(0.1));

      return point_stamped_out;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        rclcpp::get_logger("SemanticLabeling"),
        "Transformation exception: %s", ex.what());
      return {};
    }
  }

  /// \brief Prints details of unique door points
  ///
  /// \param none
  /// \return none
  void print_unique_door_points()
  {
    for (const auto & pair : unique_door_points_) {
      const auto & point = pair.first;
      int marker_id = pair.second;
      RCLCPP_INFO(
        rclcpp::get_logger("SemanticLabeling"),
        "Door point - x: %f, y: %f, z: %f, Marker id: %d",
        point.x, point.y, point.z, marker_id);
    }
  }

  /// \brief Prints details of unique table points
  ///
  /// \param none
  /// \return none
  void print_unique_table_points()
  {
    for (const auto & pair : unique_table_points_) {
      const auto & point = pair.first;
      int marker_id = pair.second;
      RCLCPP_INFO(
        rclcpp::get_logger("SemanticLabeling"),
        "Table point - x: %f, y: %f, z: %f, Marker id: %d",
        point.x, point.y, point.z, marker_id);
    }
  }

  /// \brief Checks if two points are approximately equal within a threshold
  ///
  /// \param p1 - first point to compare
  /// \param p2 - second point to compare
  /// \return true if points are approximately equal, false otherwise
  bool equal_points(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
  {
    return std::abs(p1.x - p2.x) < threshold_ &&
           std::abs(p1.y - p2.y) < threshold_ &&
           std::abs(p1.z - p2.z) < threshold_;
  }

  // Declare private variables
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr door_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr table_marker_pub_;
  rclcpp::Publisher<landmark_manager::msg::SemanticPoint>::SharedPtr semantic_door_pub_;
  rclcpp::Publisher<landmark_manager::msg::SemanticPoint>::SharedPtr semantic_table_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr door_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr table_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  int rate_;
  double threshold_;
  visualization_msgs::msg::MarkerArray door_marker_array_;
  visualization_msgs::msg::MarkerArray table_marker_array_;
  int door_marker_id_;
  int table_marker_id_;
  bool new_door_point_received_;
  bool new_table_point_received_;

  std::vector<std::pair<geometry_msgs::msg::Point, int>> unique_door_points_;
  std::vector<std::pair<geometry_msgs::msg::Point, int>> unique_table_points_;
  std::set<int> labeled_door_marker_ids_;
  std::set<int> labeled_table_marker_ids_;
};

/// \brief The main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SemanticLabeling>());
  rclcpp::shutdown();
  return 0;
}

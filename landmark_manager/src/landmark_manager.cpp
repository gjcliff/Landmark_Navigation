/// \file
/// \brief This node provides a service for saving landmarks, which are
/// displayed as markers. It also provides services for canceling navigation and
/// navigating to landmarks.
///
/// PUBLISHES:
///     /landmark_markers (visualization_msgs::msg::MarkerArray): sphere markers
///     that represent
///                                                               landmarks as
///                                                               well as text
///                                                               markers
/// SERVERS:
///     save_landmark (landmark_manager::srv::SaveLandmark): saves the landmark
///     name and position navigate_to_landmark
///     (landmark_manager::srv::NavigateToLandmark): sends a specific landmark
///                                                                       as a
///                                                                       goal
///     cancel_navigation (std_srvs::srv::SetBool): cancels the navigation
/// CLIENTS:
///     navigate_to_pose (nav2_msgs::action::NavigateToPose): navigates from a
///     starting point to a
///                                                           goal

#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "landmark_manager/srv/navigate_to_landmark.hpp"
#include "landmark_manager/srv/save_landmark.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

/// \brief Provides functionalities for saving landmarks, canceling navigation,
/// and navigating to landmarks
class LandmarkManager : public rclcpp::Node {
public:
  LandmarkManager() : Node("landmark_manager") {
    // Initializes variables for the publisher, timer, services, and action
    // client
    declare_parameter("rate", 200);
    declare_parameter("load_saved_landmarks", false);
    declare_parameter("landmarks_file_name", "");
    get_parameter("rate", rate_);
    get_parameter("load_saved_landmarks", load_saved_landmarks_);
    get_parameter("landmarks_file_name", landmarks_file_name_);

    navigation_canceled_ = false;
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/landmark_markers", 10);
    timer_ =
        create_wall_timer(std::chrono::milliseconds(1000 / rate_),
                          std::bind(&LandmarkManager::timer_callback, this));
    save_service_ = create_service<landmark_manager::srv::SaveLandmark>(
        "save_landmark",
        std::bind(&LandmarkManager::save_landmark_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    navigate_service_ =
        create_service<landmark_manager::srv::NavigateToLandmark>(
            "navigate_to_landmark",
            std::bind(&LandmarkManager::navigate_to_landmark_callback, this,
                      std::placeholders::_1, std::placeholders::_2));
    cancel_service_ = create_service<std_srvs::srv::SetBool>(
        "cancel_navigation",
        std::bind(&LandmarkManager::cancel_navigation_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");
  }

private:
  /// \brief Callback function for the timer. Clears and republishes markers.
  ///
  /// \param none
  /// \return none
  void timer_callback() {
    // Clear the marker array to prevent duplicate markers
    marker_array_.markers.clear();

    if (!landmarks_file_name_.empty()) {
      add_landmarks();
    }

    marker_pub_->publish(marker_array_);
  }

  /// \brief Callback function for the save_landmark service. Saves a new
  /// landmark.
  ///
  /// \param request - name and position of a landmark
  /// \param response - not being used
  /// \return none
  void save_landmark_callback(
      const landmark_manager::srv::SaveLandmark::Request::SharedPtr request,
      landmark_manager::srv::SaveLandmark::Response::SharedPtr /*response*/) {
    std::string file_path = get_landmarks_file_path();
    YAML::Node node;

    if (file_exists(file_path)) {
      node = YAML::LoadFile(file_path);
    }

    // Save the new landmark's name and coordinates
    node[request->name]["x"] = request->x;
    node[request->name]["y"] = request->y;

    // Open the file for writing
    std::ofstream file(file_path);

    // Write to the file and close it
    file << node;
    file.close();
  }

  /// \brief Callback function for the navigate_to_landmark service. Navigates
  /// to a specific landmark.
  ///
  /// \param request - name of the landmark
  /// \param response - not being used
  /// \return none
  void navigate_to_landmark_callback(
      const landmark_manager::srv::NavigateToLandmark::Request::SharedPtr
          request,
      landmark_manager::srv::NavigateToLandmark::Response::
          SharedPtr /*response*/) {
    std::string file_path = get_landmarks_file_path();

    if (!file_exists(file_path)) {
      RCLCPP_WARN(rclcpp::get_logger("LandmarkManager"),
                  "No landmarks file found");
      return;
    }

    YAML::Node node = YAML::LoadFile(file_path);

    if (!node[request->name]) {
      RCLCPP_WARN(rclcpp::get_logger("LandmarkManager"), "Landmark not found");
      return;
    }

    // Send the landmark as a goal
    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = get_clock()->now();
    goal.pose.pose.position.x = node[request->name]["x"].as<double>();
    goal.pose.pose.position.y = node[request->name]["y"].as<double>();

    // Set up options for sending the goal
    auto send_goal_options = rclcpp_action::Client<
        nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
        std::bind(&LandmarkManager::handle_navigation_result, this,
                  std::placeholders::_1);

    // Send the goal
    action_client_->async_send_goal(goal, send_goal_options);
  }

  /// \brief Handles the result of navigation action
  ///
  /// \param result - result of the navigation action
  /// \return none
  void handle_navigation_result(
      const rclcpp_action::ClientGoalHandle<
          nav2_msgs::action::NavigateToPose>::WrappedResult &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(rclcpp::get_logger("LandmarkManager"),
                  "Navigation succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(rclcpp::get_logger("LandmarkManager"), "Navigation failed");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(rclcpp::get_logger("LandmarkManager"),
                  "Navigation was canceled");
      navigation_canceled_ = true;
      break;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("LandmarkManager"),
                   "Unknown result code");
      break;
    }
  }

  /// \brief Callback function for the cancel_navigation service. Cancels an
  /// ongoing navigation.
  ///
  /// \param request - not being used
  /// \param response - response indicating whether the cancellation was
  /// successful
  /// \return none
  void cancel_navigation_callback(
      const std_srvs::srv::SetBool::Request::SharedPtr /*request*/,
      std_srvs::srv::SetBool::Response::SharedPtr response) {
    // Reset the cancellation status
    navigation_canceled_ = false;

    // Check if the action client is available
    if (!action_client_ || !action_client_->action_server_is_ready()) {
      RCLCPP_WARN(rclcpp::get_logger("LandmarkManager"),
                  "Action server not available");
      response->success = false;
      response->message = "Action server not available";
      return;
    }

    // Cancel all goals
    action_client_->async_cancel_all_goals();

    response->success = true;
    response->message = "Navigation canceled successfully";
  }

  /// \brief Adds landmarks to the marker array for visualization
  ///
  /// \param none
  /// \return none
  void add_landmarks() {
    // std::string file_path = get_landmarks_file_path();
    std::string file_path =
        std::string(CMAKE_SOURCE_DIR) + "/landmarks/" + landmarks_file_name_;

    if (!file_exists(file_path)) {
      RCLCPP_WARN(rclcpp::get_logger("LandmarkManager"),
                  "No landmarks file found");
      return;
    }

    YAML::Node config = YAML::LoadFile(file_path);

    // Initialize a counter for the marker id
    int marker_id = 0;

    // Iterate through each landmark in the yaml file
    for (const auto &landmark : config) {
      // Add a marker for each landmark
      visualization_msgs::msg::Marker landmark_marker;
      landmark_marker.header.frame_id = "map";
      landmark_marker.header.stamp = get_clock()->now();
      landmark_marker.id = marker_id;
      landmark_marker.type = visualization_msgs::msg::Marker::SPHERE;
      landmark_marker.action = visualization_msgs::msg::Marker::ADD;
      landmark_marker.pose.position.x = landmark.second["x"].as<double>();
      landmark_marker.pose.position.y = landmark.second["y"].as<double>();
      landmark_marker.scale.x = 0.5;
      landmark_marker.scale.y = 0.5;
      landmark_marker.scale.z = 0.5;
      landmark_marker.color.b = 1.0;
      landmark_marker.color.a = 1.0;

      marker_array_.markers.push_back(landmark_marker);

      // Add a text marker for the landmark name
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = "map";
      text_marker.header.stamp = get_clock()->now();
      text_marker.id = marker_id + 1000; // Offset to avoid id clashes
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.text = landmark.first.as<std::string>();
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.pose.position.x = landmark.second["x"].as<double>() + 0.5;
      text_marker.pose.position.y = landmark.second["y"].as<double>();
      text_marker.pose.position.z = 0.6;
      text_marker.scale.z = 0.5;
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      text_marker.color.a = 1.0;

      marker_array_.markers.push_back(text_marker);

      marker_id++;
    }
  }

  /// \brief Checks if a file exists
  ///
  /// \param name - path to the file
  /// \return true if the file exists, false otherwise
  bool file_exists(const std::string &name) {
    return std::filesystem::exists(name);
  }

  /// \brief Gets the file path of landmarks.yaml
  ///
  /// \param none
  /// \return file path
  std::string get_landmarks_file_path() {
    std::string package_path =
        ament_index_cpp::get_package_share_directory("landmark_manager");
    return package_path + "/config/landmarks.yaml";
  }

  // Declare private variables
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<landmark_manager::srv::SaveLandmark>::SharedPtr save_service_;
  rclcpp::Service<landmark_manager::srv::NavigateToLandmark>::SharedPtr
      navigate_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr cancel_service_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      action_client_;

  int rate_;
  bool load_saved_landmarks_;
  visualization_msgs::msg::MarkerArray marker_array_;
  bool navigation_canceled_;
  std::string landmarks_file_name_;
};

/// \brief The main function
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandmarkManager>());
  rclcpp::shutdown();
  return 0;
}

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "isc_nav/pure_pursuit.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

namespace isc_nav
{
class PurePursuitNode : public rclcpp::Node
{
public:
  PurePursuit m_tracker;
  explicit PurePursuitNode(rclcpp::NodeOptions options);

private:
  Path m_path;
  Point3D pose;
  std::string robot_frame;
  std::string map_frame;

  float lookahead_distance{1.5f};
  float desired_linear_velocity{1.0f};
  float desired_angular_velocity{1.0f};
  bool m_path_is_initialized;
  bool is_initialized;

	// TF
	std::shared_ptr<tf2_ros::TransformListener> transform_listener;
	std::unique_ptr<tf2_ros::Buffer> tf_buffer;
	float tf_timeout;

	/**
	* @brief A path callback function to get the path and initialize path
	* @param msg Shared ptr to the path msg recieved from the global planner
	*/
	void path_callback(const nav_msgs::msg::Path::SharedPtr msg);

	/**
	* @brief Main Driver of the Pure Pursuit Ros2 wrapper. Constructs
	* the linear, angular velocities as well as lookahead point and 
	* publishes it out
	*/
	void compute_velocity();

	/**
	* @brief A Timer Callback function to allow for Dynamic params
	* @return None
	*/
	void param_update();

	/**
	* @brief Convert the ros2 path message to a Point3D path
	* @param msg Shared ptr fo the path msg recieved from the global planner
	* @return A Path Vector of Point3D poses
	*/
	Path to_path(const nav_msgs::msg::Path::SharedPtr msg);

	/**
	* @brief Helper Function to convert a ros pose to Point3D pose
	* @param pose geometry_msgs::msg::PoseStamped pose 
	* @return Point3D pose
	*/
	Point3D to_point3d(const geometry_msgs::msg::PoseStamped pose);

	/**
	* @brief A Function to get the pose of the robot 
	* @return The Pose of the robot in Point3D
	*/
	Point3D get_pose();

	// Publishers/Subscribers
	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr carrot_publisher;
	rclcpp::TimerBase::SharedPtr param_update_timer;
};
}  // namespace isc_nav
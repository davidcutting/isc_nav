#include "isc_nav/pure_pursuit_node.hpp"

namespace isc_nav
{
using namespace std::chrono_literals;

PurePursuitNode::PurePursuitNode(rclcpp::NodeOptions options)
  : Node("pure_pursuit", options), m_tracker(m_path, m_lookahead_distance), m_path_is_initialized(false)
{
  path_subscription = this->create_subscription<nav_msgs::msg::Path>(
    "/navigation/path", 10,
    std::bind(&PurePursuitNode::path_callback, this, std::placeholders::_1));

  velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10);

  carrot_publisher = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "/lookahead_point", 10);

  this->declare_parameter<float>("m_lookahead_distance", 0.1f);
  this->declare_parameter<std::string>("robot_frame", "base_footprint");
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<bool>("m_path_is_initialized", false);
  this->declare_parameter("tf_timeout", 0.03);

  tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  param_update_timer = this->create_wall_timer(
    1000ms, std::bind(&PurePursuitNode::param_update, this)
  );
}

void PurePursuitNode::param_update()
{
  this->get_parameter("m_lookahead_distance", m_lookahead_distance);
  this->get_parameter("tf_timeout", tf_timeout);
}

void PurePursuitNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  m_path = to_path(msg);
  m_tracker.reset_path(m_path);

  if(!m_path_is_initialized) 
  {
    RCLCPP_INFO(this->get_logger(), "Got Path:)");
    m_path_is_initialized = true;
  }
}

Path PurePursuitNode::to_path(const nav_msgs::msg::Path::SharedPtr msg)
{
  Path pure_pursuit_path;
  for(auto& p: msg->poses)
  {
    pure_pursuit_path.push_back(to_point3d(p));
  }
  return pure_pursuit_path;
}

Point3D PurePursuitNode::to_point3d(const geometry_msgs::msg::PoseStamped pose)
{
  return Point3D(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
}

Point3D PurePursuitNode::get_pose()
{
  Point3D pose;
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped = tf_buffer->lookupTransform(map_frame, robot_frame, this->get_clock()->now(),
      rclcpp::Duration::from_seconds(tf_timeout));
  } 
  catch (tf2::TransformException & ex) {
    RCLCPP_INFO(
    this->get_logger(), "Could not transform %s to %s: %s",
    map_frame.c_str(), robot_frame.c_str(), ex.what());
  }
  return pose;
}

}  // namespace isc_nav

int main(int argc, char *argv[])
{
  rclcpp::init(argc,argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto pp_node = std::make_shared<isc_nav::PurePursuitNode>(options);
  exec.add_node(pp_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}

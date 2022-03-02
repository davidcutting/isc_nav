#include "isc_nav/pure_pursuit_node.hpp"

namespace isc_nav
{
PurePursuitNode::PurePursuitNode(rclcpp::NodeOptions options)
  : Node("pure_pursuit", options), m_tracker(m_path, m_lookahead_distance), m_path_is_initialized(false)
{
  path_subscription = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10,
    std::bind(&PurePursuitNode::path_callback, this, std::placeholders::_1));

  velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10);

  this->declare_parameter<float>("m_lookahead_distance", 0.1f);
  this->declare_parameter<std::string>("robot_frame", "base_footprint");
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<float>("m_lookahead_distance", 0.1f);
  this->declare_parameter<bool>("m_path_is_initialized", false);
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

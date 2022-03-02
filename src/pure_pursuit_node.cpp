#include "isc_nav/pure_pursuit_node.hpp"


namespace isc_nav
{

PurePursuitNode::PurePursuitNode(rclcpp::NodeOptions options)
  : Node("pure_pursuit", options), m_tracker(m_path, m_lookahead_distance), m_path_is_initialized(false)
{
  path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10,
    std::bind(&PurePursuitNode::path_callback, this, std::placeholders::_1));
   
  velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10);

  this->declare_parameter<float>("m_lookahead_distance", 0.1f);
  this->declare_parameter("robot_map_frame", "/map");
  this->declare_parameter("robot_base_frame", "/base_footprint");
  this->declare_parameter<float>("m_lookahead_distance", 0.1f);
  this->declare_parameter<bool>("m_path_is_initialized", false);
}

void PurePursuitNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  m_path = to_path(msg);
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

}  // namespace isc_nav

int main(int argc, char * argv[])
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


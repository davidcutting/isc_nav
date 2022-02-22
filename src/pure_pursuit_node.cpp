#include "isc_nav/pure_pursuit_node.hpp"

namespace isc_nav
{

PurePursuitNode::PurePursuitNode(rclcpp::NodeOptions options)
  : Node("pure_pursuit", options)
{
  path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10,
    std::bind(&PurePursuitNode::path_callback, this, std::placeholders::_1));
   
  velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10);
}

void PurePursuitNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  last_path_state_ = msg;
	std::cout << " Here lmao " << std::endl; // TODO(dcutting133): :thinking:
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


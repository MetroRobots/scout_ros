#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "ugv_sdk/mobile_robot/scout_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"
#include "scout_base/scout_messenger.hpp"

using namespace westonrobot;

std::unique_ptr<ScoutRobot> robot;

int main(int argc, char** argv)
{
  // setup ROS node
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("scout_odom");

  // Declare params
  node->declare_parameter("is_scout_mini", false);
  node->declare_parameter("port_name", std::string("can0"));
  node->declare_parameter("odom_frame", std::string("odom"));
  node->declare_parameter("base_frame", std::string("base_link"));
  node->declare_parameter("simulated_robot", false);
  node->declare_parameter("control_rate", 50);
  node->declare_parameter("odom_topic_name", std::string("odom"));
  node->declare_parameter("pub_tf", true);

  // check whether controlling scout mini
  bool is_scout_mini;
  node->get_parameter("is_scout_mini", is_scout_mini);

  std::cout << "Working as scout mini: " << is_scout_mini << std::endl;

  // check protocol version
  ProtocolDectctor detector;
  try
  {
    detector.Connect("can0");
    auto proto = detector.DetectProtocolVersion(5);
    if (proto == ProtocolVersion::AGX_V1)
    {
      std::cout << "Detected protocol: AGX_V1" << std::endl;
      robot = std::unique_ptr<ScoutRobot>(new ScoutRobot(ProtocolVersion::AGX_V1, is_scout_mini));
    }
    else if (proto == ProtocolVersion::AGX_V2)
    {
      std::cout << "Detected protocol: AGX_V2" << std::endl;
      robot = std::unique_ptr<ScoutRobot>(new ScoutRobot(ProtocolVersion::AGX_V2, is_scout_mini));
    }
    else
    {
      std::cout << "Detected protocol: UNKNOWN" << std::endl;
      return -1;
    }
    if (robot == nullptr)
      std::cout << "Failed to create robot object" << std::endl;
  }
  catch (const std::exception error)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ScoutBase"), "please bringup up can or make sure can port exist");
    rclcpp::shutdown();
  }
  ScoutROSMessenger messenger(robot.get(), node);

  // fetch parameters before connecting to rt
  std::string port_name;
  node->get_parameter("port_name", port_name);
  node->get_parameter("odom_frame", messenger.odom_frame_);
  node->get_parameter("base_frame", messenger.base_frame_);
  node->get_parameter("simulated_robot", messenger.simulated_robot_);
  node->get_parameter("control_rate", messenger.sim_control_rate_);
  node->get_parameter("odom_topic_name", messenger.odom_topic_name_);
  node->get_parameter("pub_tf", messenger.pub_tf);
  if (!messenger.simulated_robot_)
  {
    // connect to robot and setup ROS subscription
    if (port_name.find("can") != std::string::npos)
    {
      robot->Connect(port_name);
      robot->EnableCommandedMode();
      RCLCPP_INFO(rclcpp::get_logger("ScoutBase"), "Using CAN bus to talk with the robot");
    }
    else
    {
      //      robot->Connect(port_name, 115200);
      RCLCPP_INFO(rclcpp::get_logger("ScoutBase"), "Only CAN bus interface is supported for now");
    }
  }

  messenger.SetupSubscription();

  // publish robot state at 50Hz while listening to twist commands
  rclcpp::Rate rate(50);
  while (rclcpp::ok())
  {
    if (!messenger.simulated_robot_)
    {

      messenger.PublishStateToROS();
    }
    else
    {
      double linear, angular;

      messenger.GetCurrentMotionCmdForSim(linear, angular);

      messenger.PublishSimStateToROS(linear, angular);
    }
    rclcpp::spin_some(node);
    rate.sleep();
  }

  return 0;
}

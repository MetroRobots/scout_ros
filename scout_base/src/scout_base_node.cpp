#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf/transform_broadcaster.h>

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
  rclcpp::Node node(""), private_node("~");

  // check whether controlling scout mini
  bool is_scout_mini = false;
  //private_node.param<bool>("is_scout_mini", is_scout_mini, false);
  node.getParam("is_scout_mini", is_scout_mini);
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
    ros::shutdown();
  }
  ScoutROSMessenger messenger(robot.get(), &node);

  // fetch parameters before connecting to rt
  std::string port_name;
  private_node.param<std::string>("port_name", port_name, std::string("can0"));
  private_node.param<std::string>("odom_frame", messenger.odom_frame_, std::string("odom"));
  private_node.param<std::string>("base_frame", messenger.base_frame_, std::string("base_link"));
  private_node.param<bool>("simulated_robot", messenger.simulated_robot_, false);
  private_node.param<int>("control_rate", messenger.sim_control_rate_, 50);
  private_node.param<std::string>("odom_topic_name", messenger.odom_topic_name_, std::string("odom"));
  private_node.param<bool>("pub_tf", messenger.pub_tf, true);
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

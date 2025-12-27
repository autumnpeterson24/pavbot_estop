/*
E-Stop Node for Pavbot
author: Michael Stalford

** You can spin up this node right now (it will only read the default of the estop being set to true rn)

*/
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class EStopNode : public rclcpp::Node
{
public:
  EStopNode() : Node("estop_node")
  {
    // Parameters
    port = declare_parameter<std::string>("port", "/dev/ttyUSB0"); // This is is the Arduino is in the USB0 slot it's just a placeholder
    baud = declare_parameter<int>("baud", 115200);
    timeout_ms = declare_parameter<int>("timeout_ms", 500);

    estop_pub = create_publisher<std_msgs::msg::Bool>(
      "/safety/estop",
      rclcpp::QoS(1).transient_local() // a queue depth of 1 to keep the latest value :)
    );

    timer = create_wall_timer( std::chrono::milliseconds(50), std::bind(&EStopNode::update, this) // periodic timer for constant update
    );

    RCLCPP_INFO(get_logger(), "E-Stop node started"); // log that the node started :)
  }

private:
  void update()
  {
    /* TODO:
    1. Read serial data from arduino
    2. Parse data
    3. Publish std_msgs::Bool
    */

    // FAIL-SAFE DEFAULT (robot always on estop until the serial parsing is implemented ofr optimal safety)
    std_msgs::msg::Bool msg;
    msg.data = true;  // true = STOP (this will be whatever data comes from arduino)
    estop_pub->publish(msg);
  }

  std::string port; // the port that the nano is connected to
  int baud; //baud rate for serail communication
  int timeout_ms;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub;
  rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv)
{
// SPINNING UP THE ROS NODE
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EStopNode>());
  rclcpp::shutdown();
  return 0;
}

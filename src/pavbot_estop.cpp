/*
E-Stop Node for Pavbot
author: Michael Stalford

** You can spin up this node right now (it will only read the default of the estop being set to true rn)

*/
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>


class PavbotEStop : public rclcpp::Node
{
  private:

  // CLASS MEMBERS ----------------------------

  std::string port; // the port that the nano is connected to
  int baud; //baud rate for serail communication
  int timeout_ms;

  // declare the publishers here
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub;

  //delcare all timers here
  rclcpp::TimerBase::SharedPtr timer;
  


  // Function using in wall timer to update at a constant hertz
  void update() {
    /* TODO:
    1. Read serial data from arduino
    2. Parse datas
    3. Publish std_msgs::Bool
    */

    //CLCPP_INFO(get_logger(), "GPS data: test"); // temp log to show it is running
    // FAIL-SAFE DEFAULT (robot always on estop until the serial parsing is implemented ofr optimal safety)
    std_msgs::msg::Bool msg;

    msg.data = true;  // true = STOP (this will be whatever data comes from arduino)
    estop_pub->publish(msg);

  }

  void getSerial() {
    // Magic
  }

public:
// CONSTRUCTOR -----------------------------
  PavbotEStop() : Node("pavbot_estop") {
    // Parameters -----------------
    port = declare_parameter<std::string>("port", "/dev/ttyUSB0"); // This is is the Arduino is in the USB0 slot it's just a placeholder
    baud = declare_parameter<int>("baud", 115200);
    timeout_ms = declare_parameter<int>("timeout_ms", 500);

    // Publishers ------------------
    estop_pub = create_publisher<std_msgs::msg::Bool>("/safety/estop", rclcpp::QoS(1).transient_local() // a queue depth of 1 to keep the latest value :)
    );


    // timers ----------------
    timer = create_wall_timer( std::chrono::milliseconds(50), std::bind(&PavbotEStop::update, this) // periodic timer for constant update
    );


    // Logging to the screen when the node is created to make sure it is there :)
    RCLCPP_INFO(get_logger(), "E-Stop node started"); // log that the node started 
    RCLCPP_INFO(get_logger(), "Chatbot, make me a hoot chocolate!"); // fun message
  }


};


int main(int argc, char **argv) {
// SPINNING UP THE ROS NODE
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PavbotEStop>());
  rclcpp::shutdown();
  return 0;
}

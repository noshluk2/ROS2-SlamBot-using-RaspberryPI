
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
class robot_teleop_driver : public rclcpp::Node {
public:
	robot_teleop_driver() : Node("robot_controller") {
    cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&robot_teleop_driver::cmd_vel_callBack, this, _1));
    }

private:
    void cmd_vel_callBack(const geometry_msgs::msg::Twist::SharedPtr vel_msg) const
    {

      RCLCPP_INFO(this->get_logger(), "I heard: '%f'", vel_msg->linear.x);
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;



    };

int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	// setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<robot_teleop_driver>());

	rclcpp::shutdown();
	return 0;
}

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;


class Velocity_publisher : public rclcpp::Node
{
  public:
    Velocity_publisher(): Node("velocity_publisher"), count_(0)
    {
      vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      timer_ = this->create_wall_timer( 500ms, std::bind(&Velocity_publisher::timer_callback, this));

      subscriber = this->create_subscription<std_msgs::msg::String>
      ("topic", 10, std::bind(&Velocity_publisher::topic_callback,this,  _1)  );
    }

  private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = 1.0;
      vel_pub->publish(message);
    }

    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber;
    size_t count_;




};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Velocity_publisher>());
  rclcpp::shutdown();
  return 0;
}




// Example 2
// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>

// #include "rclcpp/rclcpp.hpp"
// #include "turtlesim/msg/pose.hpp"
// #include "geometry_msgs/msg/twist.hpp"

// using namespace std::chrono_literals;
// using std::placeholders::_1;


// class Go_to_goal_node : public rclcpp::Node
// {
//   public:
//     const double PI = 3.141592;
//     const double Ka = .5;
//     const double Klv = .5;
//     const double angularTolerance = .1;
//     const double distanceTolerance = .1;
//     bool waypointActive = false;

//     Go_to_goal_node(): Node("gtg_node"), count_(0)
//     {
//       vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
//       timer_ = this->create_wall_timer( 100ms, std::bind(&Go_to_goal_node::velocity_callback, this));

//       pose_sub = this->create_subscription<turtlesim::msg::Pose>
//       ("turtle1/pose", 10, std::bind(&Go_to_goal_node::pose_callback,this,  _1)  );
//     }


//   private:

//     void velocity_callback()
//     {

//       auto cmdVel = geometry_msgs::msg::Twist();
//       cmdVel.linear.x = 0.5;
//       cmdVel.angular.z = 0.5;

//       vel_pub->publish(cmdVel);

//     }

//     void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) const
//     {
//       std::cout<<msg->x<<" / "<<msg->y<<" / "<<msg->theta<<std::endl;
//     }


//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
//     rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub;
//     size_t count_;




// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<Go_to_goal_node>());
//   rclcpp::shutdown();
//   return 0;
// }
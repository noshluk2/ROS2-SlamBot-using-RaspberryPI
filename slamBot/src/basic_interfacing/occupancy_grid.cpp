#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std::chrono_literals;

class occupancyGridPublisher : public rclcpp::Node
{
public:
    occupancyGridPublisher()
        : Node("OGM_node"), count_(0)
    {
        ogm_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&occupancyGridPublisher::timer_callback, this));
    }

private:
    const int width = 100;
    const int height = 100;
    const double resolution = .1;
    void timer_callback()
    {
        auto ogm_msg = nav_msgs::msg::OccupancyGrid();
        ogm_msg.info.width = width;
        ogm_msg.info.height = height;
        ogm_msg.info.resolution = resolution;

        ogm_msg.info.origin.position.x = 0;
        ogm_msg.info.origin.position.y = 0;
        // init all cells to unknown
        ogm_msg.data.resize(ogm_msg.info.width * ogm_msg.info.height);
        for (int i = 0; i < 10000; i++)
        {
            ogm_msg.data[i] = -1;
        }
        // ogm_pub.header.stamp = rclcpp::Clock().now(); // occupancyGridPublisher.get_clock()
        ogm_pub->publish(ogm_msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr ogm_pub;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<occupancyGridPublisher>());
    rclcpp::shutdown();
    return 0;
}

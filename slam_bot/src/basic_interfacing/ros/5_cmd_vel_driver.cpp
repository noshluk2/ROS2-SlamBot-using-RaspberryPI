
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/Int16.h"
#include <pigpiod_if2.h>


using std::placeholders::_1;


// Motor Pins
const int PWM_A = 25;
const int MOTOR_A_FWD = 24;
const int MOTOR_A_REV = 23;
const int PWM_B = 4;
const int MOTOR_B_FWD = 15;
const int MOTOR_B_REV = 14;
float right_wheel;float left_wheel;
class robot_teleop_driver : public rclcpp::Node
{
public:
  robot_teleop_driver() : Node("robot_controller")
  {
    cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&robot_teleop_driver::cmd_vel_callBack, this, _1));
  }

private:

void cmd_vel_callBack( const geometry_msgs::Twist& velocity_msg){

    right_wheel = (velocity_msg.linear.x + velocity_msg.angular.z ) / 2 ;
    left_wheel = (velocity_msg.linear.x - velocity_msg.angular.z ) /2 ;
    direction();
    speed();
    if ( velocity_msg.linear.x ==0.0 & velocity_msg.angular.z ==0.0){
        stop();
    }
    Serial.print(left_wheel);Serial.print(" / ");Serial.println(right_wheel);

}

void direction(){
    gpio_write(pi, MOTOR_A_FWD, 1);
    digitalWrite(L_FORW, left_wheel >0 );
    digitalWrite(L_BACK,left_wheel < 0);
    digitalWrite(R_FORW,right_wheel > 0 );
    digitalWrite(R_BACK,right_wheel < 0);
}

void speed (){
    ledcWrite(channel_R, 200);
    ledcWrite(channel_L, 200);
}

void stop()
{


   ledcWrite(channel_R, 0);
   ledcWrite(channel_L, 0);
}

  int PigpioSetup()
  {
    char *addrStr = NULL;
    char *portStr = NULL;
    pi = pigpio_start(addrStr, portStr);
    // next 10 lines sets up our pins. Remember that high is "off"
    // and we must drive in1 or in2 low to start the output to motor
    set_mode(pi,PWM_A, PI_OUTPUT);
    set_mode(pi,MOTOR_A_FWD, PI_OUTPUT);
    set_mode(pi,MOTOR_A_REV, PI_OUTPUT);

    set_mode(pi,PWM_B, PI_OUTPUT);
    set_mode(pi,MOTOR_B_FWD, PI_OUTPUT);
    set_mode(pi,MOTOR_B_REV, PI_OUTPUT);

    gpio_write(pi, MOTOR_A_FWD, 1);
    gpio_write(pi, MOTOR_A_REV, 1);
    gpio_write(pi, MOTOR_B_FWD, 1);
    gpio_write(pi, MOTOR_B_REV, 1);

    return pi;
  }
};

int main(int argc, char *argv[])
{
  std::cout << "Starting Cmd Driver node..." << std::endl;
  // setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_teleop_driver>());

  rclcpp::shutdown();
  return 0;
}

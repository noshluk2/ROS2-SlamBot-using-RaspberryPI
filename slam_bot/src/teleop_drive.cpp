#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <pigpiod_if2.h>


const uint8_t R_PWM =  25;
const uint8_t R_BACK = 24;
const uint8_t R_FORW = 23;

const uint8_t L_PWM =  4;
const uint8_t L_BACK = 15;
const uint8_t L_FORW = 14;

const uint8_t channel_L =0;
const uint8_t channel_R= 1;

float left_wheel;
float right_wheel;

using namespace std;
int pigpio_setup()
{
    char *addrStr = NULL;
    char *portStr = NULL;
    const int pi = pigpio_start(addrStr, portStr);

    set_mode(pi,R_PWM,  PI_OUTPUT);
    set_mode(pi,R_FORW, PI_OUTPUT);
    set_mode(pi,R_BACK, PI_OUTPUT);
    set_mode(pi,L_PWM,  PI_OUTPUT);
    set_mode(pi,L_FORW, PI_OUTPUT);
    set_mode(pi,L_BACK, PI_OUTPUT);

    gpio_write(pi, R_FORW, 1);
    gpio_write(pi, R_BACK, 1);
    gpio_write(pi, L_FORW, 1);
    gpio_write(pi, L_BACK, 1);
    return pi;
}

int main(){


    int pi = PigpioSetup();
    if(PigpioSetup()>=0)
    {
     cout<<"daemon interface started ok at "<<pi<<endl;
    }
    else
    {
     cout<<"Failed to connect to PIGPIO Daemon - is it running?"<<endl;
     return -1;
    }



    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("teleop_driver");
    auto sub = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,cmdVel_to_pwm_cb);
    rclcpp::spin(node);


}



void cmdVel_to_pwm_cb( const geometry_msgs::msg::Twist::SharedPtr velocity_msg){

    right_wheel = (velocity_msg.linear.x + velocity_msg.angular.z ) / 2 ;
    left_wheel = (velocity_msg.linear.x - velocity_msg.angular.z ) /2 ;
    direction();
    speed();
    if ( velocity_msg.linear.x ==0.0 & velocity_msg.angular.z ==0.0){
        stop();
    }

}

void direction(){
    gpio_write(pi, L_FORW, left_wheel  >0 );
    gpio_write(pi, L_BACK, left_wheel  <0);
    gpio_write(pi, R_FORW, right_wheel >0 );
    gpio_write(pi, R_BACK, right_wheel <0 );
    }

void speed (){
    set_PWM_dutycycle(pi, R_PWM, 127);
    set_PWM_dutycycle(pi, L_PWM, 127);
}

void stop()
{
    set_PWM_dutycycle(pi, R_PWM, 0);
    set_PWM_dutycycle(pi, L_PWM, 0);
}
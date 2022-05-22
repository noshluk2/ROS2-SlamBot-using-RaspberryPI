
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/Int16.h"
#include <pigpiod_if2.h>


using std::placeholders::_1;
// Code Variables Defination
const int PWM_INCREMENT = 1;             // the rate pwm out can change per cycle
const double ticksPerwheelRev = 254 * 2; // 508.8; //not in use yet..just a reference for now
const double wheelRadius = .03575;       // 55.18;
const double wheelBase = .224;           // 223.8375mm actually
const double TICKS_PER_M = 1125 * 2;     // 1.1645; //1.365 is on hard floor. carpet avg is 1.1926. overall avg = 1.1645 1125.766 t/m
const int KP = 238;
const int DRIFT_MULTIPLIER = 125;
const int TURN_PWM = 54;
const int MIN_PWM = 52;
const int MAX_PWM = 100;

// Motor Pins
const int PWM_L = 4;
const int PWM_R = 25;
const int MOTOR_L_FWD = 14;
const int MOTOR_L_REV = 15;
const int MOTOR_R_FWD = 23;
const int MOTOR_R_REV = 24;
// Velocity Variabels
double leftVelocity = 0;
double rightVelocity = 0;
double leftPwmReq = 0;
double rightPwmReq = 0;
double lastCmdMsgRcvd = 0; // time of last command recieved

class robot_teleop_driver : public rclcpp::Node
{
public:
  robot_teleop_driver() : Node("robot_controller")
  {
    cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&robot_teleop_driver::cmd_vel_callBack, this, _1));
  }

private:
  void Calc_Left_Vel(const std_msgs::Int16 &lCount)
  {
    static double lastTime = 0;
    static int lastCount = 0;
    int cycleDistance = (65535 + lCount.data - lastCount) % 65535;
    if (cycleDistance > 10000)
    {
      cycleDistance = 0 - (65535 - cycleDistance);
    }
    leftVelocity = cycleDistance / TICKS_PER_M / (ros::Time::now().toSec() - lastTime);
    lastCount = lCount.data;
    lastTime = ros::Time::now().toSec();
  }

  void Calc_Right_Vel(const std_msgs::Int16 &rCount)
  {
    static double lastTime = 0;
    static int lastCount = 0;
    int cycleDistance = (65535 + rCount.data - lastCount) % 65535;
    if (cycleDistance > 10000)
    {
      cycleDistance = 0 - (65535 - cycleDistance);
    }
    rightVelocity = cycleDistance / TICKS_PER_M / (ros::Time::now().toSec() - lastTime);
    lastCount = rCount.data;
    lastTime = ros::Time::now().toSec();
  }

  void Set_Speeds(const geometry_msgs::Twist &cmdVel)
  {
    lastCmdMsgRcvd = ros::Time::now().toSec();
    int b = (cmdVel.linear.x > .025 && cmdVel.linear.x < .052) ? 45 : 40;
    leftPwmReq = KP * cmdVel.linear.x + b;
    rightPwmReq = KP * cmdVel.linear.x + b;
    cout << "iniitial pwm request (left and right same) = " << leftPwmReq << endl;
    if (cmdVel.angular.z != 0)
    {
      if (cmdVel.angular.z > .0) // standard gentle turn
      {
        leftPwmReq = -TURN_PWM;
        rightPwmReq = TURN_PWM;
      }
      else
      {
        leftPwmReq = TURN_PWM;
        rightPwmReq = -TURN_PWM;
      }
      if (abs(cmdVel.angular.z > .12)) // turn a little faster if angle is greater that .12
      {
        leftPwmReq *= 1.1;
        rightPwmReq *= 1.1;
      }
    }
    else if (abs(cmdVel.linear.x) > .0478) // should equal about pwmval of 50
    {
      static double prevDiff = 0;
      static double prevPrevDiff = 0;
      double angularVelDifference = leftVelocity - rightVelocity;                   // how much faster one wheel is actually turning
      double avgAngularDiff = (prevDiff + prevPrevDiff + angularVelDifference) / 3; // average several cycles
      prevPrevDiff = prevDiff;
      prevDiff = angularVelDifference;

      // apply corrective offset to each wheel to try and go straight
      leftPwmReq -= (int)(avgAngularDiff * DRIFT_MULTIPLIER);
      rightPwmReq += (int)(avgAngularDiff * DRIFT_MULTIPLIER);
    }

    // don't PWM values that don't do anything
    if (abs(leftPwmReq) < MIN_PWM)
    {
      leftPwmReq = 0;
    }

    if (abs(rightPwmReq) < MIN_PWM)
    {
      rightPwmReq = 0;
    }
    ////left for debugging and tweaking
    // cout<<"CMD_VEL = "<<cmdVel.linear.x<<endl;
    // cout<<"VEL, AND PWM REQ LEFT AND RIGHT "<<leftVelocity<<"    "<<leftPwmReq<<" ..... ";
    // cout<<rightVelocity<<"    "<<rightPwmReq<<endl;
  }

  void set_pin_values()
  {
    static int leftPwmOut = 0;
    static int rightPwmOut = 0;

    // if PwmReq*PwmOut is negative, that means the wheel is switching
    // directions and we should bring to a stop before switching directions
    static bool stopped = false;
    if ((leftPwmReq * leftVelocity < 0 && leftPwmOut != 0) || (rightPwmReq * rightVelocity < 0 && rightPwmOut != 0))
    {
      leftPwmReq = 0;
      rightPwmReq = 0;
    }

    // set motor driver direction pins
    if (leftPwmReq > 0) // left fwd
    {
      gpio_write(pi, MOTOR_L_REV, 1);
      gpio_write(pi, MOTOR_L_FWD, 0);
    }
    else if (leftPwmReq < 0) // left rev
    {
      gpio_write(pi, MOTOR_L_FWD, 1);
      gpio_write(pi, MOTOR_L_REV, 0);
    }
    else if (leftPwmReq == 0 && leftPwmOut == 0) // left stop
    {
      gpio_write(pi, MOTOR_L_FWD, 1);
      gpio_write(pi, MOTOR_L_REV, 1);
    }

    if (rightPwmReq > 0) // right fwd
    {
      gpio_write(pi, MOTOR_R_REV, 1);
      gpio_write(pi, MOTOR_R_FWD, 0);
    }
    else if (rightPwmReq < 0) // right rev
    {
      gpio_write(pi, MOTOR_R_FWD, 1);
      gpio_write(pi, MOTOR_R_REV, 0);
    }
    else if (rightPwmReq == 0 && rightPwmOut == 0)
    {
      gpio_write(pi, MOTOR_R_FWD, 1);
      gpio_write(pi, MOTOR_R_REV, 1);
    }

    // bump up pwm if robot is having trouble starting from stopped
    if (leftPwmReq != 0 && leftVelocity == 0)
    {
      leftPwmReq *= 1.4;
    }
    if (rightPwmReq != 0 && rightVelocity == 0)
    {
      rightPwmReq *= 1.4;
    }

    // this section increments PWM changes instead of jarring/dangeroud sudden big changes
    if (abs(leftPwmReq) > leftPwmOut)
    {
      leftPwmOut += PWM_INCREMENT;
    }
    else if (abs(leftPwmReq) < leftPwmOut)
    {
      leftPwmOut -= PWM_INCREMENT;
    }
    if (abs(rightPwmReq) > rightPwmOut)
    {
      rightPwmOut += PWM_INCREMENT;
    }
    else if (abs(rightPwmReq) < rightPwmOut)
    {
      rightPwmOut -= PWM_INCREMENT;
    }

    // cap output at max defined in constants
    leftPwmOut = (leftPwmOut > MAX_PWM) ? MAX_PWM : leftPwmOut;
    rightPwmOut = (rightPwmOut > MAX_PWM) ? MAX_PWM : rightPwmOut;

    // limit output to a low of zero
    leftPwmOut = (leftPwmOut < 0) ? 0 : leftPwmOut;
    rightPwmOut = (rightPwmOut < 0) ? 0 : rightPwmOut;

    // write the pwm values tot he pins
    set_PWM_dutycycle(pi, PWM_L, leftPwmOut);
    set_PWM_dutycycle(pi, PWM_R, rightPwmOut);

    cout << "PWM OUT LEFT AND RIGHT            " << leftPwmOut << "           " << rightPwmOut << endl
         << endl;
  }

  int PigpioSetup()
  {
    char *addrStr = NULL;
    char *portStr = NULL;
    pi = pigpio_start(addrStr, portStr);
    // next 10 lines sets up our pins. Remember that high is "off"
    // and we must drive in1 or in2 low to start the output to motor
    set_mode(pi, PWM_L, PI_OUTPUT);
    set_mode(pi, MOTOR_L_FWD, PI_OUTPUT);
    set_mode(pi, MOTOR_L_REV, PI_OUTPUT);
    set_mode(pi, PWM_R, PI_OUTPUT);
    set_mode(pi, MOTOR_R_FWD, PI_OUTPUT);
    set_mode(pi, MOTOR_R_REV, PI_OUTPUT);

    gpio_write(pi, MOTOR_L_FWD, 1); // initializes motor off
    gpio_write(pi, MOTOR_L_REV, 1); // initializes motor off
    gpio_write(pi, MOTOR_R_FWD, 1); // initializes motor off
    gpio_write(pi, MOTOR_R_REV, 1); // initializes motor off

    return pi;
  }
};

int main(int argc, char *argv[])
{
  std::cout << "Starting offboard control node..." << std::endl;
  // setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_teleop_driver>());

  rclcpp::shutdown();
  return 0;
}

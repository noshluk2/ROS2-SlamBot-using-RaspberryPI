#include <math.h>
#include <ESP8266WiFi.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

                                          ///// Defining the PWM maximum and minimum range
#define PWMRANGEs 1020 // if you type not "s" it will pick basic pwm range for me it took 255 for the arduino causing problems
#define PWM_MIN 300

                                          ///// Initializing the functions that we will be using through the code 
void setupPins();
void setupSerial();
void setupWiFi();
bool rosConnected();
void onTwist(const geometry_msgs::Twist &msg);
float mapPwm(float x, float out_min, float out_max);

                                          ///// All pins defining
const uint8_t R_PWM =  D4;
const uint8_t R_BACK = D7;
const uint8_t R_FORW = D8;

const uint8_t L_PWM =  D3;
const uint8_t L_BACK = D5;
const uint8_t L_FORW = D6; 

                                           ///// WIFI setup
const char* ssid     = "paradise1";
const char* password = "pass#123";
bool _connected = false;


IPAddress server(192,168,10,2); // this is the ip of computer on which ROSCORE is running
const uint16_t serverPort = 11411; // ip supplied with port for communication

                                           //// Setting up arduino ROS communication handle 
ros::NodeHandle node;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &onTwist); // keyboard input subscriber


                                           //// Generic structure of arduino IDE programming

void setup()
{ Serial.begin(115200);
  setupPins();
  setupWiFi();
  node.getHardware()->setConnection(server); // connecting esp8266 module to ROSCORE
  node.initNode();
  node.subscribe(sub);// starting to subscribe the desired topic
}

                                           //// Difining Functionality of all pins

void setupPins()
{ /// calling this function from setup();
  pinMode(L_PWM,  OUTPUT);
  pinMode(L_FORW, OUTPUT);
  pinMode(L_BACK, OUTPUT);
  pinMode(R_PWM,  OUTPUT);
  pinMode(R_FORW, OUTPUT);
  pinMode(R_BACK, OUTPUT);
  stop(); // making robot to not move
}

                                            //// Connecting esp to common ROSCORE wifi                               
void setupWiFi()
{  /// calling this function from setup();
   Serial.begin(115200);

  // We start by connecting to a WiFi network

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
     would try to act as both a client and an access-point and could cause
     network-issues with your other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());


}
                                           //// Stop function to stop the robot movement
void stop()
{
  digitalWrite(L_FORW, 0);
  digitalWrite(L_BACK, 0);
  digitalWrite(R_FORW, 0);
  digitalWrite(R_BACK, 0);
  analogWrite(L_PWM,   0);
  analogWrite(R_PWM,   0);
}

                                           //// Starting the commands dealing mathemetically

void onTwist(const geometry_msgs::Twist &msg)
{ Serial.println("Messaeg is recieved");
  
  float x = max(min(msg.linear.x, 1.0f), -1.0f);// minimum value (-1) maximum value (1)
  float z = max(min(msg.angular.z, 1.0f), -1.0f);// minimum value (-1) maximum value (1)
  
                                          /// Transforming linear and angular velocities to speed for the LEFT RIGHT MOTORS
  float l = (msg.linear.x - msg.angular.z) / 2;
  float r = (msg.linear.x + msg.angular.z) / 2;
                                          ///According to the speed derived we get pwm for motors
  uint16_t lPwm = mapPwm(fabs(l), 450, 1050);
  uint16_t rPwm = mapPwm(fabs(r), 450, 1050);
  
  digitalWrite(L_FORW, l > 0);
  digitalWrite(R_FORW, r > 0);
  
  digitalWrite(L_BACK, l < 0);
  digitalWrite(R_BACK, r < 0);
  Serial.println(l);
  Serial.println(r);
  
  analogWrite(L_PWM, lPwm);
  analogWrite(R_PWM, rPwm);
  Serial.println(lPwm);
  Serial.println(rPwm);
  
}



                                           //// Main loop just spins on the back subscribing node gets the work done
void loop()
{
  node.spinOnce();
  delay(10);
}


float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}

#include <vector>
#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/imu.hpp"

#define ACC 0x33
#define GYO 0x34
#define DEG 0x35

static int16_t acc_x = 0, acc_y = 0, acc_z = 0, gyo_x = 0, gyo_y = 0, gyo_z = 0, deg_x = 0, deg_y = 0, deg_z = 0;
static bool run = false;

#define DEG2RAD( a ) ( (a) * (M_PI/180.0f) )
#define COS(a) cos(DEG2RAD(a))
#define SIN(a) sin(DEG2RAD(a))

using namespace std;

auto imu = std::make_shared<sensor_msgs::msg::Imu>();

rclcpp::TimerBase::SharedPtr Serial_timer;
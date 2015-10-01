#define ROS_MASTER_URI	"http:://localhost:11311"
#define ROS_ROOT	"opt/ros/hydro/share/ros"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <boost/assign/list_of.hpp>

#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <stdlib.h>
#include "stdio.h"

#define DEFAULT_BAUDRATE 460800
#define BAUDMACRO	B460800
#define DEFAULT_SERIALPORT "/dev/ttyUSB0"
#define FPS	40



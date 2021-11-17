#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <sensor_msgs/JointState.h>

#include <ur3e_mrc/position.h>
#include <ur3e_mrc/gripper_input.h>

#include <boost/math/constants/constants.hpp>

#include <ur_msgs/IOStates.h>

ros::Subscriber sub_js;
ros::Subscriber sub_io;

ros::Publisher pos_pub;
ros::Publisher grip_grasp_pub;
ros::Publisher grip_inp_pub;
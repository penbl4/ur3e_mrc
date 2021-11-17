#include <ur3e_mrc/ur3e_mrc_enme480_topics.h>

void jsCallback(const sensor_msgs::JointState &msg)
{
  ur3e_mrc::position pos_msg;
  pos_msg.position = {msg.position[2] + boost::math::constants::pi<double>() / 2, msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]};

  pos_msg.isReady = true;
  pos_pub.publish(pos_msg);
}

void ioCallback(const ur_msgs::IOStates &msg)
{
  std_msgs::Bool grip_grasp_msg;
  ur3e_mrc::gripper_input grip_inp_msg;

  grip_grasp_msg.data = msg.digital_in_states[0].state;

  if (msg.digital_in_states[0].state)
    grip_inp_msg.DIGIN = 1;
  else
    grip_inp_msg.DIGIN = 0;

  grip_inp_msg.AIN0 = msg.analog_in_states[0].state;
  grip_inp_msg.AIN1 = 0.0;

  grip_grasp_pub.publish(grip_grasp_msg);
  grip_inp_pub.publish(grip_inp_msg);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ur3e_mrc_enme480_topics");

  ros::NodeHandle nh;

  sub_js = nh.subscribe("joint_states", 10, jsCallback);
  ROS_INFO("Subscribed to joint states");

  sub_io = nh.subscribe("/ur_hardware_interface/io_states", 10, ioCallback);
  ROS_INFO("Subscribed to IO states");

  pos_pub = nh.advertise<ur3e_mrc::position>("ur3/position", 10);
  grip_grasp_pub = nh.advertise<std_msgs::Bool>("gripper/grasping", 10);
  grip_inp_pub = nh.advertise<ur3e_mrc::gripper_input>("ur3/gripper_input", 10);

  ros::spin();

  return 0;
}
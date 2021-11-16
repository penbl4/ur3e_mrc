#include <ur3e_mrc/ur3e_mrc_enme480.h>

#define N_JOINTS 6
#define CTRL_TO_RUN "scaled_pos_joint_traj_controller"

sig_atomic_t volatile g_request_shutdown = 0;
bool volatile ur3e_isReady = true;
bool volatile grip_isON = false;

UR3eArm::UR3eArm()
{
  trajectory_client_ = new TrajectoryClient("scaled_pos_joint_traj_controller/follow_joint_trajectory", true);
  ROS_INFO("Waiting for the joint_trajectory_action server");
  init_status_ = trajectory_client_->waitForServer(ros::Duration(20.0));
  if (init_status_)
    ROS_INFO("Connected to the action server");
  else
    ROS_ERROR("Failed to connect to the action server");

  pos_pub = nh_.advertise<ur3e_mrc::position>("ur3/position", 10);
  grip_grasp_pub = nh_.advertise<std_msgs::Bool>("gripper/grasping", 10);
  grip_inp_pub = nh_.advertise<ur3e_mrc::gripper_input>("ur3/gripper_input", 10);

  sub_js_ = nh_.subscribe("joint_states", 10, &UR3eArm::jsCallback, this);
  ROS_INFO("Subscribed to joint states");

  sub_comm_ = nh_.subscribe("ur3/command", 10, &UR3eArm::commCallback, this);
  ROS_INFO("Subscribed to ur3 command");

  sub_io_ = nh_.subscribe("/ur_hardware_interface/io_states", 10, &UR3eArm::ioCallback, this);
  ROS_INFO("Subscribed to ur3 command");

  // Controller manager service to switch controllers
  ctrl_manager_srv_ = nh_.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
  // Controller manager service to list controllers
  ctrl_list_srv_ = nh_.serviceClient<controller_manager_msgs::ListControllers>("controller_manager/list_controllers");

  ctrl_manager_srv_.waitForExistence();
  ctrl_list_srv_.waitForExistence();

  //making sure the proper controller is running
  ctrlRunCheck();

  // Controller manager service to manage the io controller
  io_client_srv_ = nh_.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");
  io_client_srv_.waitForExistence();

  ROS_INFO("Turning ON the vacuum generator");
  ioCtrl(12, 1);
  ioCtrl(1, 0); // making sure the gripper is OFF


}

UR3eArm::~UR3eArm()
{
  delete trajectory_client_;
  nh_.shutdown();
}

void UR3eArm::ctrlRunCheck()
{
  ROS_INFO("Running scaled_pos_joint_traj_controller check");
  controller_manager_msgs::ListControllers list_srv;
  ctrl_list_srv_.call(list_srv);
  for (auto &controller : list_srv.response.controller)
  {
    if (controller.name == CTRL_TO_RUN)
    {
      if (controller.state != "running")
      {
        ROS_INFO("Attempting to activate scaled_pos_joint_traj_controller");
        std::vector<std::string> ctrl_to_run; // = {CTRL_TO_RUN};
        ctrl_to_run.push_back(CTRL_TO_RUN);

        controller_manager_msgs::SwitchController srv;
        srv.request.strictness = srv.request.STRICT;
        srv.request.start_controllers = ctrl_to_run;
        if (!ctrl_manager_srv_.call(srv))
        {
          ROS_ERROR("Could not activate scaled_pos_joint_traj_controller");
        }
      }
    }
  }
}

void UR3eArm::ioCtrl(unsigned short int numPin, unsigned short int valPin) 
{
  ur_msgs::SetIO io_service;
  io_service.request.fun = 1; // 1 is digital output
  io_service.request.pin = numPin; // Pin number
  io_service.request.state = valPin;   // State, as far as I know it can be also voltage

  if (io_client_srv_.call(io_service))
  {
    ROS_INFO("IO request executed.");
  }
  else
  {
    ROS_ERROR("IO request failed.");
  }  
}

void UR3eArm::sendTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
{
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  trajectory_client_->sendGoal(goal);
  ROS_INFO_STREAM("Send joint trajectory goal to server successfully!");
}

void UR3eArm::initGoal(control_msgs::FollowJointTrajectoryGoal &goal)
{
  // First, the joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("shoulder_pan_joint");
  goal.trajectory.joint_names.push_back("shoulder_lift_joint");
  goal.trajectory.joint_names.push_back("elbow_joint");
  goal.trajectory.joint_names.push_back("wrist_1_joint");
  goal.trajectory.joint_names.push_back("wrist_2_joint");
  goal.trajectory.joint_names.push_back("wrist_3_joint");
  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].velocities.resize(N_JOINTS);
  goal.trajectory.points[0].velocities = {}; // Initialize with all zeros
  goal.trajectory.points[0].accelerations.resize(N_JOINTS);
  goal.trajectory.points[0].accelerations = {}; // Initialize with all zeros
}

bool UR3eArm::initStatus()
{
  return init_status_;
}

//   goal.trajectory.points[index].positions = {0, -1.57, -1.57, 0, 0, 0};
//   goal.trajectory.points[index].positions = {0.2, -1.57, -1.57, 0, 0, 0};
//   goal.trajectory.points[index].positions = {-0.5, -1.57, -1.2, 0, 0, 0};
//   goal.trajectory.points[index].positions = {0.0, 0.0, 0.0, 0, 0, 0};

actionlib::SimpleClientGoalState UR3eArm::getState()
{
  return trajectory_client_->getState();
}

void UR3eArm::jsCallback(const sensor_msgs::JointState &msg)
{
  ur3e_mrc::position pos_msg;
  pos_msg.position = {msg.position[2] + boost::math::constants::pi<double>()/2, msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]};

  pos_msg.isReady = ur3e_isReady;
  pos_pub.publish(pos_msg);
}

void UR3eArm::commCallback(const ur3e_mrc::command &msg)
{
  // ROS_INFO("Got ur3e command");
  if (msg.destination.size() != N_JOINTS)
  {
    ROS_INFO("WARNNING: In commCallback- received command size is not 6");
    return;
  }

  // if(msg->io_0 == true ) {
  // 	sprintf(buf,"set_digital_out(0,True)\n");
  // 	sendCMD = buf;
  // } else {
  // 	sprintf(buf,"set_digital_out(0,False)\n");
  // 	sendCMD = buf;
  // }

  // initiate the goal variable
  control_msgs::FollowJointTrajectoryGoal goal;
  ur3e_isReady = false;

  if (msg.io_0 && !grip_isON)
  {
    ioCtrl(1, 1); // turning the gripper ON
    grip_isON = true;
  }
  else if (!msg.io_0 && grip_isON)
  {
    ioCtrl(1, 0); // turning the gripper OFF
    grip_isON = false;
  }  


  initGoal(goal);

  // velocity = msg->v;
  // acceleration = msg->a;
  // if (acceleration < 0.1) {
  // 	ROS_INFO("Acceleration too low setting to 0.1 rad/s^2");
  // 	acceleration = 0.1;
  // }
  // if (acceleration > 4.0) {
  // 	ROS_INFO("Acceleration too high setting to 4.0 rad/s^2");
  // 	acceleration = 4.0;
  // }

  // if (velocity < 0.1) {
  // 	ROS_INFO("Velocity too low setting to 0.1 rad/s");
  // 	velocity = 0.1;
  // }
  // if (velocity > 4.0) {
  // 	ROS_INFO("Velocity too high setting to 4.0 rad/s");
  // 	velocity = 4.0;
  // }

  // fill the positions of the goal
  goal.trajectory.points[0].positions.resize(N_JOINTS);
  goal.trajectory.points[0].positions = msg.destination;
  goal.trajectory.points[0].positions[0] -= boost::math::constants::pi<double>()/2;
  goal.trajectory.points[0].time_from_start = ros::Duration(2.0);
  // pt.time_from_start = get_duration(data.destination, data.v)



  sendTrajectory(goal);
  while (!getState().isDone() && ros::ok())
  {
    usleep(50000);
  }

  ur3e_isReady = true;

  // global gripper_is_on

  // // Vacuum Gripper

  // if data.io_0 and (not gripper_is_on):
  //     gripper_on_srv = rospy.ServiceProxy('/gripper/on', Empty)
  //     gripper_on_srv()
  //     gripper_is_on = True
  // elif (not data.io_0) and gripper_is_on:
  //     gripper_off_srv = rospy.ServiceProxy('/gripper/off', Empty)
  //     gripper_off_srv()
  //     gripper_is_on = False

  // cmd_pub.publish(jt)
}

void UR3eArm::ioCallback(const ur_msgs::IOStates &msg)
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

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  g_request_shutdown = 1;
  // ros::shutdown();
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ur3e_mrc_enme480", ros::init_options::NoSigintHandler);

  UR3eArm ur3e_mrc_var;

  signal(SIGINT, mySigintHandler);

  // Override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);  

  if (!ur3e_mrc_var.initStatus())
    ros::shutdown();

  // if (ur3e_mrc_var.initStatus())
  //   ros::spin();
  // else
  //   ros::shutdown();

  while (!g_request_shutdown)
  {
    // Do non-callback stuff
    ros::spinOnce();
    usleep(50000);
  }

  //shutting down procedure
  ROS_INFO("Turning OFF the vacuum generator");
  ur3e_mrc_var.ioCtrl(12, 0);
  ros::shutdown();

  return 0;
}
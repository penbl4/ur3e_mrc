#include <ros/ros.h>
#include <signal.h>
#include <ros/xmlrpc_manager.h>

#include <std_msgs/String.h>
// #include <sensor_msgs/JointState.h>

#include <ur3e_mrc/command.h>

#include <boost/math/constants/constants.hpp>
// #include <boost/shared_ptr.hpp>
// #include <boost/make_shared.hpp>

// #include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
// #include <control_msgs/FollowJointTrajectoryGoal.h>
#include <actionlib/client/simple_action_client.h>

#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>

#include <ur_msgs/SetIO.h>
#include <ur_msgs/IOStates.h>
#include <ur_robot_driver/hardware_interface.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

class UR3eArm
{
private:
    ros::NodeHandle nh_;
    TrajectoryClient *trajectory_client_;
    ros::Subscriber sub_comm_;

    ros::ServiceClient ctrl_manager_srv_;
    ros::ServiceClient ctrl_list_srv_;
    ros::ServiceClient io_client_srv_;

    void ctrlRunCheck();

    bool init_status_ = false;

public:
    UR3eArm();
    ~UR3eArm();
    void commCallback(const ur3e_mrc::command &msg);

    void initGoal(control_msgs::FollowJointTrajectoryGoal &goal);
    void sendTrajectory(control_msgs::FollowJointTrajectoryGoal goal);

    void ioCtrl(unsigned short int numPin, unsigned short int valPin);

    // control_msgs::FollowJointTrajectoryGoal ur3eTrajectory(); //const ur3e_mrc::position &msg
    bool initStatus();
    actionlib::SimpleClientGoalState getState();

protected:
};
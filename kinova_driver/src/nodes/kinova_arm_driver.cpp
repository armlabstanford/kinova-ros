//============================================================================
// Name        : kinova_arm_driver.cpp
// Author      : WPI, Clearpath Robotics
// Version     : 0.5
// Copyright   : BSD
// Description : A ROS driver for controlling the Kinova Kinova robotic manipulator arm
//============================================================================

#include "kinova_driver/kinova_api.h"
#include "kinova_driver/kinova_arm.h"
#include "kinova_driver/kinova_tool_pose_action.h"
#include "kinova_driver/kinova_joint_angles_action.h"
#include "kinova_driver/kinova_fingers_action.h"
#include "kinova_driver/kinova_joint_trajectory_controller.h"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <kinova_driver/kinova_comm.h>
#include <kinova_driver/kinova_ros_types.h>
#include <kinova_msgs/FingerPosition.h>

namespace kinova
{
    class FingerController
{
public:
    FingerController(ros::NodeHandle& nh, KinovaComm& kinova_comm, boost::recursive_mutex& api_mutex) : nh_(nh),
          finger_mode_(0), kinova_comm_(kinova_comm), api_mutex_(api_mutex_)    
    {
        joy_sub_ = nh_.subscribe("/joy", 10, &FingerController::joyCallback, this);
        mode_sub_ = nh_.subscribe("/finger_mode", 10, &FingerController::modeCallback, this);
        finger_pos_sub_ = nh_.subscribe("/j2n6s300_driver/out/finger_position", 10, &FingerController::fingerPosCallback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;
    ros::Subscriber mode_sub_;
    ros::Subscriber finger_pos_sub_;
    kinova::FingerAngles finger_position_;
    int finger_mode_;
    KinovaComm& kinova_comm_;
    boost::recursive_mutex& api_mutex_;

    void modeCallback(const std_msgs::Int32::ConstPtr& msg)
    {
        finger_mode_ = msg->data;
    }

    void fingerPosCallback(const kinova_msgs::FingerPosition::ConstPtr& msg)
    {
        if(finger_mode_ != 1)
            return; 

        // just keep the values
        finger_position_.Finger1 = msg->finger1;
        finger_position_.Finger2 = msg->finger2;
        finger_position_.Finger3 = msg->finger3;
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        if(finger_mode_ != 1)
            return; // only move fingers if mode is 1

        kinova::FingerAngles angles;
        // only if joy is moved considerably
        if (msg->axes[1] > -0.1 && msg->axes[1] < 0.1)
            return;
        
        angles.Finger1 = finger_position_.Finger1 - 6800*(msg->axes[1])/5; // scale joystick [-1,1] to [0,6800]
        angles.Finger2 = finger_position_.Finger2 - 6800*(msg->axes[1])/5; // scale joystick [-1,1] to [0,6800]
        angles.Finger3 = finger_position_.Finger3 - 6800*(msg->axes[1])/5; // scale joystick [-1,1] to [0,6800]

        // clip between 0 and 6800
        if (angles.Finger1 < 0)
            angles.Finger1 = 0;
        if (angles.Finger1 > 6800)
            angles.Finger1 = 6800;
        if (angles.Finger2 < 0)
            angles.Finger2 = 0;
        if (angles.Finger2 > 6800)
            angles.Finger2 = 6800;
        if (angles.Finger3 < 0)
            angles.Finger3 = 0;
        if (angles.Finger3 > 6800)
            angles.Finger3 = 6800;  

        try
        {
            kinova_comm_.setFingerPositions(angles, 0.0, true);
        }
        catch(const KinovaCommException& e)
        {
            ROS_ERROR("Failed to move fingers: %s", e.what());
        }
    }
};

} // namespace kinova

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinova_arm_driver");
    ros::NodeHandle nh("~");
    boost::recursive_mutex api_mutex;

    bool is_first_init = true;
    std::string kinova_robotType = "";
    std::string kinova_robotName = "";

    // Retrieve the (non-option) argument:
    if ( (argc <= 1) || (argv[argc-1] == NULL) ) // there is NO input...
    {
        std::cerr << "No kinova_robotType provided in the argument!" << std::endl;
        return -1;
    }
    else // there is an input...
    {
        kinova_robotType = argv[argc-1];
        ROS_INFO("kinova_robotType is %s.", kinova_robotType.c_str());
        if (!nh.getParam("robot_name", kinova_robotName))
        {
          kinova_robotName = kinova_robotType;
        }
        ROS_INFO("kinova_robotName is %s.", kinova_robotName.c_str());
    }

    kinova::KinovaComm kinova_comm(nh, api_mutex, true, "j2n6s300");
    kinova::FingerController controller(nh, kinova_comm, api_mutex);


    while (ros::ok())
    {
        try
        {
            kinova::KinovaComm comm(nh, api_mutex, is_first_init,kinova_robotType);
            kinova::KinovaArm kinova_arm(comm, nh, kinova_robotType, kinova_robotName);
            kinova::KinovaPoseActionServer pose_server(comm, nh, kinova_robotType, kinova_robotName);
            kinova::KinovaAnglesActionServer angles_server(comm, nh);
            kinova::KinovaFingersActionServer fingers_server(comm, nh);
            kinova::JointTrajectoryController joint_trajectory_controller(comm, nh);
            ros::spin();
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM(e.what());
            kinova::KinovaAPI api;
            boost::recursive_mutex::scoped_lock lock(api_mutex);
            api.closeAPI();
            ros::Duration(1.0).sleep();
        }

        is_first_init = false;
    }
    return 0;
}

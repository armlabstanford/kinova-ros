#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <kinova_driver/kinova_comm.h>
#include <kinova_driver/kinova_ros_types.h>
#include <boost/thread/recursive_mutex.hpp>

using namespace kinova;

class FingerController
{
public:
    FingerController(ros::NodeHandle& nh, KinovaComm& kinova_comm, boost::recursive_mutex& api_mutex) : nh_(nh),
          finger_mode_(0), kinova_comm_(kinova_comm), api_mutex_(api_mutex_)    
    {
        joy_sub_ = nh_.subscribe("/joy", 10, &FingerController::joyCallback, this);
        mode_sub_ = nh_.subscribe("/finger_mode", 10, &FingerController::modeCallback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;
    ros::Subscriber mode_sub_;
    int finger_mode_;
    KinovaComm& kinova_comm_;
    boost::recursive_mutex& api_mutex_;

    void modeCallback(const std_msgs::Int32::ConstPtr& msg)
    {
        finger_mode_ = msg->data;
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        if(finger_mode_ != 1)
            return; // only move fingers if mode is 1

        kinova::FingerAngles angles;
        angles.Finger1 = msg->axes[0] * 6800; // scale joystick [-1,1] to [0,6800]
        angles.Finger2 = msg->axes[0] * 6800;
        angles.Finger3 = msg->axes[0] * 6800;

        try
        {
            {
                boost::recursive_mutex::scoped_lock lock(api_mutex_);
                kinova_comm_.setFingerPositions(angles, 0.0, true);
            }
        }
        catch(const KinovaCommException& e)
        {
            ROS_ERROR("Failed to move fingers: %s", e.what());
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_fingers_node");
    ros::NodeHandle nh;
    boost::recursive_mutex api_mutex;  // declared in kinova_driver.cpp or kinova_arm.cpp

    kinova::KinovaComm kinova_comm(nh, api_mutex, true, "j2n6s300");
    FingerController controller(nh, kinova_comm, api_mutex);

    ros::spin();
    return 0;
}

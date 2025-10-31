#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <kinova_driver/kinova_ros_types.h>
#include <kinova_msgs/JointVelocity.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <ros/console.h>
#include <eigen3/Eigen/Dense>

#include <unordered_map>

static const int QUEUE_LENGTH = 1;

class JointCommander
{
public:
    JointCommander();
    void jointsCallback(const sensor_msgs::JointState& msg);
    void jointsVelCallback(const kinova_msgs::JointVelocity& msg);
    void mode_switch(void);

private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber joints_vel_sub_;
	std::vector<std::string> joint_names_;
    Eigen::VectorXd joint_values_;
    ros::Publisher joints_vel_pub_;
    ros::Duration dt_ = ros::Duration(0.1); // 10 Hz
    ros::Time last_time_ = ros::Time::now();
};

JointCommander::JointCommander() : joint_values_(Eigen::VectorXd::Zero(6)), joint_names_({"j2n6s300_joint_1", "j2n6s300_joint_2", "j2n6s300_joint_3", "j2n6s300_joint_4", "j2n6s300_joint_5", "j2n6s300_joint_6"})
{
    nh_ = ros::NodeHandle();
    pose_sub_ = nh_.subscribe("/wrapped_joint_states", QUEUE_LENGTH, &JointCommander::jointsCallback, this);
    joints_vel_sub_ = nh_.subscribe("/desired_joint_speeds", QUEUE_LENGTH, &JointCommander::jointsVelCallback, this);
    joints_vel_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/j2n6s300_driver/trajectory_controller/command", QUEUE_LENGTH);
    
    // sleep(10.0);

    moveit::planning_interface::MoveGroupInterface group("arm");

    ROS_WARN("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_WARN("End effector link: %s", group.getEndEffectorLink().c_str());
    
    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

}

void JointCommander::jointsVelCallback(const kinova_msgs::JointVelocity& msg)
{

    std::vector<float> joint_speeds = {
    msg.joint1,
    msg.joint2,
    msg.joint3,
    msg.joint4,
    msg.joint5,
    msg.joint6
    };

	trajectory_msgs::JointTrajectory joints_vel_msg;
	joints_vel_msg.header.frame_id = "base_link";
	joints_vel_msg.header.stamp = ros::Time::now();
	joints_vel_msg.joint_names.clear();
	trajectory_msgs::JointTrajectoryPoint point;
	for (int i=0; i< joint_names_.size(); i ++ )
	{
        ros::Time now = ros::Time::now();
        double dt = (now - last_time_).toSec();
        last_time_ = now;

		joints_vel_msg.joint_names.push_back(joint_names_[i]);
        point.positions.push_back(joint_values_[i] + dt*joint_speeds[i]);
        point.velocities.push_back(joint_speeds[i]);

		// if(joy_touch_){ //if joystick is being touched right now 
		// 	if(!joy_cont_touched_){ //if it wasn't being touched before, then update the joint goal from the current joint values
		// 		joint_goal_prev_[i] = joint_values_[i] + dt*joints_vel_msg.values_(i);
		// 		point.positions.push_back(joint_goal_prev_[i]); //and command a motion to that goal
		// 	} else { //if it has been touched already and is still being touched, then don't update from current joint values, just add the current increment to the previous joint goal! We don't want goals to be affected by gravity
		// 		point.positions.push_back(joint_goal_prev_[i] + dt*joints_vel_msg.values_(i));
		// 		joint_goal_prev_[i] = joint_goal_prev_[i] + dt*joints_vel_msg.values_(i); // increment joint_goal_previous for the next iteration
		// 	}
		// } else if (joy_ever_touched_){ // if joystick has been released, just keep publishing whatever was the last goal 
		// 	point.positions.push_back(joint_goal_prev_[i]);
		// }
	}
	point.time_from_start = dt_;
	joints_vel_msg.points.push_back(point);
	joints_vel_pub_.publish(joints_vel_msg);

}

void JointCommander::jointsCallback(const sensor_msgs::JointState& msg)
{
	std::unordered_map<std::string, float> stringFloatMap;
	for (int i=0; i< msg.name.size(); i++){
			stringFloatMap.insert(std::make_pair(msg.name[i], msg.position[i]));
	};
	for(int i=0; i< joint_names_.size(); i++){
			auto it = stringFloatMap.find(joint_names_[i]);
			if (it != stringFloatMap.end()) {
					joint_values_[i] = it->second;
			} else {
					std::cout << "String not found." << std::endl;
			}
	}
    std::cout << "Joint values: ";
    for (int i=0; i< joint_names_.size(); i++){
        std::cout << joint_names_[i] << ": " << joint_values_[i] << ", ";
    }
    std::cout << std::endl;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "send_joint_speeds");
  JointCommander JointCommanderObj;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::waitForShutdown();
  return 0;
}

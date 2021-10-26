// Usage:
// Run "source devel/setup.bash" on a new terminal
// Run "roslaunch coborg_move AM_Interpolator_launch.launch"

//Includes
#include <string>
#include <math.h>
#include <algorithm>
#include "ros/ros.h"
// #include <tf/tf.h>
// #include <tf/transform_listener.h>
#include <Eigen/Dense>
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
// #include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
// #include "gb_visual_detection_3d_msgs/goal_msg.h" // Change to darknet_ros_3d/goal_message when switching to arm branch
// #include <moveit_msgs/DisplayTrajectory.h>
// #include <moveit_msgs/PlanningScene.h>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/planning_pipeline/planning_pipeline.h>
// #include <moveit/planning_interface/planning_interface.h>
// #include <moveit/planning_scene/planning_scene.h>
// #include <moveit/kinematic_constraints/utils.h>
// #include <moveit_msgs/DisplayRobotState.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
#include <sensor_msgs/JointState.h>

// HEBI Includes
// #include "hebi_cpp_api/lookup.hpp"
// #include "hebi_cpp_api/group.hpp"
// #include "hebi_cpp_api/group_command.hpp"
// #include "hebi_cpp_api/group_feedback.hpp"
// #include "hebi_cpp_api/trajectory.hpp"

// Global Variable Declarations
std::vector<sensor_msgs::JointState> current_trajectory;
const int HZ = 10;

// Subscriber Callbacks
void new_trajectory_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    return;
}

int main(int argc, char** argv)
{	
    // Initialize ROS
	ros::init(argc, argv, "AM_interpolator");

	// Construct node
	ros::NodeHandle node_handle;

    // Configure the loop frequency (Hertz):
    ros::Rate loop_rate(HZ);

	// Initialize Publishers
	ros::Publisher joint_state_pub = node_handle_ptr->advertise<sensor_msgs::JointState::ConstPtr>("/move_group/fake_controller_joint_states", 1, true);
	ros::Duration(0.5).sleep();

	// Initialize Subscribers
	ros::Subscriber new_trajectory_sub = node_handle.subscribe("/new_trajectory", 1, new_trajectory_callback);

	std::cout<<"Publishers and subscribers initialized"<<std::endl;
	std::cout<<"Looping and ready"<<std::endl;

    while (ros::ok())
    {
        ros::spinOnce();
        joint_state_pub.publish(current_trajectory[0]);
        loop_rate.sleep();
    }
    return 0;
}
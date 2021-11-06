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
#include "std_msgs/String.h"
// #include "geometry_msgs/TransformStamped.h"
// #include "geometry_msgs/PoseStamped.h"
// #include "gb_visual_detection_3d_msgs/goal_msg.h" // Change to darknet_ros_3d/goal_message when switching to arm branch
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <sensor_msgs/JointState.h>

// HEBI Includes
// #include "hebi_cpp_api/lookup.hpp"
// #include "hebi_cpp_api/group.hpp"
// #include "hebi_cpp_api/group_command.hpp"
// #include "hebi_cpp_api/group_feedback.hpp"
// #include "hebi_cpp_api/trajectory.hpp"

// States
// 0 == waiting
// 1 == executing

// Global Variable Declarations
const unsigned int joint_num = 4;
// std::string name[joint_num] = {"base_1", "shoulder_2", "elbow_3", "wrist_4"};
std::vector<std::string> name = {"base_1", "shoulder_2", "elbow_3", "wrist_4"};
std::string frame_id = "/world";
// moveit_msgs::RobotTrajectory new_trajectory;
// moveit_msgs::RobotTrajectory prev_trajectory;
trajectory_msgs::JointTrajectory new_trajectory;
trajectory_msgs::JointTrajectory prev_trajectory;
sensor_msgs::JointState next_point;
std::vector<std::vector<double>> positions;
// double positions[][name.size()];
// double positions[][sizeof(name)/sizeof(*name)];
// double positions[1][4];
unsigned int cur_pos = 0;
unsigned int working_pos = 0;
// double* cur_pos_ptr;
// unsigned int cur_step = sizeof(positions[0]);
const int HZ = 20;
double time_step = 1.0/HZ;
unsigned int state = 0;

// Subscriber Callback
void new_trajectory_callback(const moveit_msgs::MotionPlanResponse::ConstPtr& msg)
{
    std::cout<<"Trajectory Received"<<std::endl;
    if (state == 0)
    {
        std::cout<<"Restarting trajectory"<<std::endl;
        prev_trajectory = msg->trajectory.joint_trajectory;
        new_trajectory = msg->trajectory.joint_trajectory;
        std::cout<<"new_trajectory is: "<<new_trajectory<<std::endl;
        std::cout<<"Trajectories saved"<<std::endl;
        positions.clear();
        cur_pos = 0;
        working_pos = 0;
        std::cout<<"Starting trajectory loop"<<std::endl;
        std::cout<<"new_trajectory.points.size() is: "<<new_trajectory.points.size()<<std::endl;
        for (unsigned int ii = 1; ii < new_trajectory.points.size(); ii++)
        {
            // Store the position indice of the point in the acceleration parameter
            std::cout<<"ii is: "<<ii<<std::endl;
            std::cout<<"working_pos is: "<<working_pos<<std::endl;
            // new_trajectory.points[ii-1].accelerations[0] = 0.15;
            // std::cout<<"Performed new_trajectory.points[ii-1].acceleartions[0] = 0.15"<<std::endl;
            // new_trajectory.points[ii-1].accelerations.at(0) = 0.15;
            // std::cout<<"Performed new_trajectory.points[ii-1].accelerations.at(0) = 0.15"<<std::endl;
            new_trajectory.points[ii-1].accelerations.push_back((double)working_pos);
            // new_trajectory.points[ii-1].accelerations[0] = (double)working_pos;
            std::cout<<"Stored working_pos: "<<working_pos<<std::endl;
            double double_num_pts = (new_trajectory.points[ii].time_from_start - new_trajectory.points[ii-1].time_from_start).toSec()/time_step;
            unsigned int num_pts = (unsigned int) double_num_pts;
            std::cout<<"new_trajectory.points[ii].time_from_start is: "<<new_trajectory.points[ii].time_from_start<<std::endl;
            std::cout<<"new_trajectory.points[ii-1].time_from_start is: "<<new_trajectory.points[ii-1].time_from_start<<std::endl;
            std::cout<<"(new_trajectory.points[ii].time_from_start - new_trajectory.points[ii-1].time_from_start).toSec() is: "<<(new_trajectory.points[ii].time_from_start - new_trajectory.points[ii-1].time_from_start).toSec()<<std::endl;
            std::cout<<"time_step is: "<<time_step<<std::endl;
            std::cout<<"(new_trajectory.points[ii].time_from_start - new_trajectory.points[ii-1].time_from_start).toSec()/time_step is: "<<(new_trajectory.points[ii].time_from_start - new_trajectory.points[ii-1].time_from_start).toSec()/time_step<<std::endl;
            std::cout<<"double_num_pts is: "<<double_num_pts<<std::endl;
            std::cout<<"num_pts is: "<<num_pts<<std::endl;
            for (unsigned int jj = 0; jj < num_pts; jj++)
            {
                std::vector<double> pos_worker_0;
                for (unsigned int kk = 0; kk < joint_num; kk++)
                {
                    pos_worker_0.push_back(((new_trajectory.points[ii].positions[kk] - new_trajectory.points[ii-1].positions[kk]) * jj / num_pts) + new_trajectory.points[ii-1].positions[kk]);
                    // positions[working_pos][kk] = ((new_trajectory.points[ii].positions[kk] - new_trajectory.points[ii-1].positions[kk]) * jj / num_pts) + new_trajectory.points[ii-1].positions[kk];
                }
                std::cout<<"pos_worker_0 filled"<<std::endl;
                positions.push_back(pos_worker_0);
                std::cout<<"pos_worker_0 pushed back"<<std::endl;
                working_pos += 1;
            }
        }
        std::cout<<"New trajectory successfully created"<<std::endl;
        state = 1;
    }
    else
    {
        std::cout<<"Updating trajectory"<<std::endl;
        prev_trajectory = new_trajectory;
        new_trajectory = msg->trajectory.joint_trajectory;
        bool divergence_point_found = 0;
        // Find the divergence point of the two trajectories
        std::cout<<"Finding divergence point"<<std::endl;
        for (unsigned int ii = 0; ii < prev_trajectory.points.size(); ii++)
        {
            // Check whether the first point of the new trajectory overlaps a point of the old trajectory
            // Also ensure that this point hasn't already been executed
            if (std::equal(std::begin(prev_trajectory.points[ii].positions), std::end(prev_trajectory.points[ii].positions), std::begin(new_trajectory.points[0].positions)) && prev_trajectory.points[ii].accelerations[0] > cur_pos)
            {
                working_pos = (int)prev_trajectory.points[ii].accelerations[0];
                divergence_point_found = 1;
                std::cout<<"Divergence point found"<<std::endl;
                break;
            }
        }
        if (divergence_point_found == 1)
        {
            for (unsigned int ii = 1; ii < new_trajectory.points.size(); ii++)
            {
                // Store the position indice of the point in the acceleration parameter
                new_trajectory.points[ii-1].accelerations[0] = (double)working_pos;
                std::cout<<"Store working_pos: "<<working_pos<<std::endl;
                unsigned int num_pts = (new_trajectory.points[ii].time_from_start - new_trajectory.points[ii-1].time_from_start).toSec()/time_step;
                for (unsigned int jj = 0; jj < num_pts; jj++)
                {
                    std::vector<double> pos_worker_1;
                    for (unsigned int kk = 0; kk < pos_worker_1.size(); kk++)
                    {
                        pos_worker_1.push_back(((new_trajectory.points[ii].positions[kk] - new_trajectory.points[ii-1].positions[kk]) * jj / num_pts) + new_trajectory.points[ii-1].positions[kk]);
                        // positions[working_pos][kk] = ((new_trajectory.points[ii].positions[kk] - new_trajectory.points[ii-1].positions[kk]) * jj / num_pts) + new_trajectory.points[ii-1].positions[kk];
                    }
                    if (working_pos < positions.size())
                    {
                        std::cout<<"Adjusting position vector"<<std::endl;
                        positions.at(working_pos) = pos_worker_1;
                    }
                    else
                    {
                        std::cout<<"Adding to position vector"<<std::endl;
                        positions.push_back(pos_worker_1);
                    }
                    working_pos += 1;
                }
            }
        }
    }
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
    ros::Publisher trajectory_feedback_pub = node_handle.advertise<std_msgs::Int32>("/interpolator_success", 1, true);
	ros::Publisher joint_state_pub = node_handle.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 1, true);
	ros::Duration(0.5).sleep();

	// Initialize Subscribers
	ros::Subscriber new_trajectory_sub = node_handle.subscribe("/new_trajectory", 1, new_trajectory_callback);

	std::cout<<"Publishers and subscribers initialized"<<std::endl;

    // Initialize next_point message
    next_point.name = name;
    next_point.header.frame_id = frame_id;
	std::cout<<"Looping and ready"<<std::endl;

    while (ros::ok())
    {
        ros::spinOnce();
        std::cout<<"Loop start is: "<<ros::Time::now().toSec()<<std::endl;
        if (state == 1)
        {
            std::cout<<"state == 1"<<std::endl;
            // Fill in JointState message
            next_point.header.stamp = ros::Time::now();
            next_point.position = positions[cur_pos];
            std::cout<<"next_point filled in"<<std::endl;
            cur_pos += 1;
            joint_state_pub.publish(next_point);
            if (cur_pos > positions.size())
            {
                next_point.header.seq = 0;
                std_msgs::Int32 success;
                success.data = 1;
                trajectory_feedback_pub.publish(success);
                std::cout<<"Finished trajectory"<<std::endl;
                state = 0;
            }
            next_point.header.seq += 1;
        }
        std::cout<<"Loop end is: "<<ros::Time::now().toSec()<<std::endl;
        loop_rate.sleep();
        std::cout<<"Loop after sleep is: "<<ros::Time::now().toSec()<<std::endl;
    }
    return 0;
}
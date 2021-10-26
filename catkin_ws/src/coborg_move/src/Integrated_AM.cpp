// Usage:
// Run "source devel/setup.bash" on every terminal
// Run "catkin_make"
// In a separate terminal, run "roscore"
// In a separate terminal, run "roslaunch main_state_machine actuated_mani_main.launch"
// In a separate terminal, run "roslaunch coborg_move KDC_find_hebi_moveit_planner.launch"
// In a separate terminal, run "roslaunch coborg_move KDC_actuated_test_launch.launch"
// Optional topic echoes
	// Publisher topics
	// In a separate terminal, run "rostopic echo /state_input"
	// In a separate terminal, run "rostopic echo /desired_hebi_efforts"
	// In a separate terminal, run "rostopic echo /move_group/display_planned_path"
	// Subscriber topics
	// In a separate terminal, run "rostopic echo /goal"
	// In a separate terminal, run "rostopic echo /state_output"
	// In a separate terminal, run "rostopic echo /execute_trajectory/feedback"
	// In a separate terminal, run "rostopic echo /hebi_joints"

//Includes
#include <string>
#include <math.h>
#include <algorithm>
#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "gb_visual_detection_3d_msgs/goal_msg.h" // Change to darknet_ros_3d/goal_message when switching to arm branch
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
#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include "hebi_cpp_api/trajectory.hpp"

// TO DO:
// Update goal_tolerance_angle with the actual tolerance of the end-effector
// Consider adding an adjusting tolerance for plans in the while loop
// Consider allowing the stitching and planning time offsets to be lowered if they were previously increased

// States:
// -1 temporary state used to block actions
// 0 waiting
// 1 ready to receive goal
// 2 approaching target
// 3 naive push
// 4 stabilization/impedance control
// 5 naive pull
// 6 return to home

////////  REMOVE  /////////
int num_attempts = 0;
int num_max_attempts = 20;
///////////////////////////

// Initialize variables
// HEBI Initializations
// Eigen::Vector3d motor_joints;
// sensor_msgs::JointState publishState;
// geometry_msgs::Vector3 torqueVect;
std::shared_ptr<hebi::Group> group;
std::vector<std::string> families = {"01-base", "02-shoulder","03-elbow","04-wrist"};
std::vector<std::string> names = {"base_1", "shoulder_2", "elbow_3", "wrist_4"};
// MoveIt Global Pointers
planning_scene::PlanningScenePtr* psmPtr;
planning_pipeline::PlanningPipelinePtr planning_pipeline_global_ptr;
moveit_visual_tools::MoveItVisualTools* visual_tools_ptr;
const moveit::core::JointModelGroup* joint_model_group;
moveit::planning_interface::MoveGroupInterface* move_group_ptr;
moveit::core::RobotStatePtr* robot_state_ptr;
moveit_msgs::MotionPlanResponse* response_ptr;
// ROS Initializations
ros::Publisher* state_input_pub_ptr;
ros::Publisher* display_publisher_ptr;
// Planning Initializations
int state = 6;
std_msgs::Int32 status;
const std::string PLANNING_GROUP = "dof_4_lowerlonger_arm";
std::string move_group_planner_id = "RRTConnect";
// Initialize Goal and Transform Variables
// tf::TransformListener* listener_ptr;
// tf::StampedTransform odom_tf_goal;
// double tf_delay_threshold = 0.1;
// ros::Time most_recent_transform;
// std::string goal_frame_name = "/world";
// std::string global_origin_frame_name = "/t265_odom_frame";
// std::string default_camera_frame_name = "/d400_link";
// ros::Time goal_time;
// Eigen::Vector4d homogeneous_goal;
// Eigen::Vector3d goal_normal;
// Eigen::Vector3d goal_normal_x_axis;
// Eigen::Vector3d goal_normal_y_axis;
// Eigen::Vector3d goal_normal_z_axis;
// Eigen::Matrix3d goal_normal_rotation_matrix;
// Eigen::Vector3d odom_tf_goal_translation;
// Eigen::Matrix3d odom_tf_goal_rotation_matrix;
// Eigen::Matrix4d odom_tf_goal_homogeneous_matrix;
// Eigen::Vector3d global_goal;
// Eigen::Vector3d global_goal_normal;
std::vector<double> goal_tolerance_pose_default {0.05, 0.05, 0.05};
std::vector<double> goal_tolerance_pose = goal_tolerance_pose_default;
std::vector<double> goal_tolerance_angle_default {10, 0.79, 0.79};
std::vector<double> goal_tolerance_angle = goal_tolerance_angle_default;
double goal_tolerance_pose_adjustment = 0.025;
double goal_tolerance_pose_adjusted_threshold = 0.1;
float goal_offset = 0.15;
// tf::StampedTransform odom_tf_current;
// Eigen::Vector3d odom_tf_current_translation;
// Eigen::Matrix3d odom_tf_current_rotation_matrix;
// Eigen::Matrix4d odom_tf_current_homogeneous_matrix;
// Initialize Planning Variables
std::string moveit_planner = "RRTConnect";
double moveit_planning_time_initial_goal = 2.0;
double moveit_planning_time_return_home = 2.0;
unsigned int num_attempts_initial_goal = 5;
unsigned int num_attempts_stitching = 1;
unsigned int num_attempts_return_home = 5;
planning_interface::MotionPlanRequest plan_req;
planning_interface::MotionPlanResponse plan_res;
moveit_msgs::MotionPlanResponse prev_plan_res;
geometry_msgs::PoseStamped goal_pose;
moveit::planning_interface::MoveItErrorCode moveitSuccess;
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
double plan_execution_start_delay = 0.4;
ros::Time plan_start;
ros::Time plan_end;
std::string end_effector_name = "end_link/INPUT_INTERFACE";
unsigned int stitching_loop_number = 0; // REMOVE
double planning_time_offset_default = 0.3;
double planning_time_offset = planning_time_offset_default;
double planning_time_offset_increase_rate = 0.05;
double stitching_time_offset_default = 0.1;
double stitching_time_offset = stitching_time_offset_default;
double stitching_time_offset_increase_rate = 0.05;
ros::Time stitch_plan_start;
double desired_plan_start_time;
bool trajectory_start_point_success;
int trajectory_start_point;
int new_plan_origin;
bool new_plan_origin_found = 0;
ros::Duration stitching_plan_time_diff;
// Initialize Naive Push Variables
float naive_push_speed = 0.03;
Eigen::VectorXd desired_velocity(6);
float naive_push_force_threshold = 5;
Eigen::VectorXd end_effector_force(6);
// Initialize Stabilization Variables
Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
std::vector<double> hebiJointAngles(4);
std::vector<double> hebiJointAngVelocities(4);
// Eigen::Vector3d current_hebi_joints;
Eigen::Isometry3d end_effector_state;
Eigen::VectorXd impedance_goal(6);
Eigen::VectorXd impedance_global_goal(6);
tf::StampedTransform odom_tf_current_impedance;
Eigen::Vector3d odom_tf_current_translation_impedance;
Eigen::Matrix3d odom_tf_current_rotation_matrix_impedance;
Eigen::Matrix4d odom_tf_current_homogeneous_matrix_impedance;
double impedance_goal_offset = 0.1;
Eigen::Vector3d impedance_position_gain;
Eigen::Vector3d impedance_derivative_gain;
Eigen::Vector3f position_error;
Eigen::Vector3f velocity_error;
Eigen::VectorXd desired_cartesian_forces(6);
Eigen::Vector3d jointVelocityVect;
Eigen::Vector3d taskSpaceVelocityVect;
Eigen::MatrixXd wristJacobian;
Eigen::VectorXd desiredTorquesEigen;
sensor_msgs::JointState desired_torques;
float baseMaxTorque = 2.0;
float elbowMaxTorque = 2.0;
float wristMaxTorque = 2.0;
// Initialize Naive Pull Variables
double naive_pull_offset = 0.1;
Eigen::Isometry3d naive_pull_starting_pose;
Eigen::Isometry3d naive_pull_current_pose;
// Initialize Return to Home Variables
std::vector<double> joint_group_positions = { 0.0, -1.8, -1.9, -2.1 };
// std::vector<double> joint_group_positions = { 0.0, -1.8, -2.2, -2.4 };

// Define Functions
void update_impedance_goal()
{
	// // Get the current transform from the ODOM frame to the world frame
	// listener_ptr -> lookupTransform(goal_frame_name, global_origin_frame_name, ros::Time::now(), odom_tf_current_impedance);
	// odom_tf_current_translation_impedance << odom_tf_current_impedance.getOrigin().getX(), odom_tf_current_impedance.getOrigin().getY(), odom_tf_current_impedance.getOrigin().getZ();
	// // Store the tf::Quaternion
	// tf::Quaternion tfQuat_impedance;
	// tfQuat_impedance = odom_tf_current_impedance.getRotation();
	// // Convert the tf::Quaternion to an Eigen vector
	// Eigen::VectorXd odom_tf_current_quaternion_impedance_worker(4);
	// odom_tf_current_quaternion_impedance_worker << tfQuat_impedance.x(), tfQuat_impedance.y(), tfQuat_impedance.z(), tfQuat_impedance.w();
	// // Convert the Eigen vector to an Eigen quaternion
	// Eigen::Quaterniond odom_tf_current_quaternion_impedance(odom_tf_current_quaternion_impedance_worker(3),odom_tf_current_quaternion_impedance_worker(0),odom_tf_current_quaternion_impedance_worker(1),odom_tf_current_quaternion_impedance_worker(2));
	// Eigen::Quaterniond odom_tf_current_quaternion_impedance_normalized;
	// odom_tf_current_quaternion_impedance_normalized = odom_tf_current_quaternion_impedance.normalized();
	// odom_tf_current_rotation_matrix_impedance = odom_tf_current_quaternion_impedance_normalized.toRotationMatrix();	
	// odom_tf_current_homogeneous_matrix_impedance << odom_tf_current_rotation_matrix_impedance, odom_tf_current_translation_impedance, 0, 0, 0, 1;
	// // Use the transform to convert the received global impedance goal to the current, local goal in the world frame
	// goal_normal = odom_tf_current_rotation_matrix_impedance * global_goal_normal;
	// // Translate the global goal into homogeneous coordinates
	// Eigen::VectorXd impedance_global_goal_homogeneous_worker(4);
	// impedance_global_goal_homogeneous_worker << impedance_global_goal(0,2), 1;
	// // Transform the global impedance goal into the local frame
	// impedance_goal(0,2) = (odom_tf_current_homogeneous_matrix * impedance_global_goal_homogeneous_worker)(0,2);
	// return;
}

// Define subscriber callbacks
// Local Goal Update
void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal_pose = *msg;
    if (state == 1)
    {
        state = -1;
    }
    return;
}
// Camera goal callback
// void camera_goal_callback(const gb_visual_detection_3d_msgs::goal_msg::ConstPtr& msg)
// {
// 	std::cout<<"Current state is: "<<state<<std::endl;
// 	if (state == 1)
// 	{
// 		std::cout<<"Goal Message Received"<<std::endl;
// 		// Block other camera inputs
// 		state = -1;
// 		// Reset the planning and stitching time offsets from the previous action
// 		planning_time_offset = planning_time_offset_default;
// 		stitching_time_offset = stitching_time_offset_default;
// 		// Set the time from the message header
// 		goal_time = msg->header.stamp;
// 		// Convert the received goal to eigen
// 		std::cout<<"Populating goals"<<std::endl;
// 		goal_normal << msg->normal_x, msg->normal_y, msg->normal_z;
// 		homogeneous_goal << msg->x, msg->y, msg->z, 1.0;
// 		// Get the transform from the local frame to the ODOM frame
// 		std::cout<<"Listening to transform"<<std::endl;
// 		while(true){
// 			try{
// 				listener_ptr -> waitForTransform(global_origin_frame_name, msg->header.frame_id, goal_time, ros::Duration(3.0));
// 				std::cout<<"Waited for transform"<<std::endl;
// 				listener_ptr -> lookupTransform(global_origin_frame_name, msg->header.frame_id, goal_time, odom_tf_goal);
// 				std::cout<<"Looked up transform"<<std::endl;
// 				break;
// 			}
// 			catch(...){
// 				std::cout<<"Current ROS time is: "<<ros::Time::now()<<std::endl;
// 				std::cout<<"Message time is: "<<goal_time<<std::endl;
// 				if (goal_time.toSec() < 1.0)
// 				{
// 					/////////////// REMOVE once goal getter sends good frames
// 					state = 1;
// 					return;
// 				}
// 				ros::Duration(3).sleep();
// 			}
// 		}																	
// 		std::cout<<"Populating odom_tf_goal_translation"<<std::endl;
// 		odom_tf_goal_translation << odom_tf_goal.getOrigin().getX(), odom_tf_goal.getOrigin().getY(), odom_tf_goal.getOrigin().getZ();
// 		// Store the tf::Quaternion
// 		tf::Quaternion tfQuat_initial;
// 		tfQuat_initial = odom_tf_goal.getRotation();
// 		// Convert the tf::Quaternion to an Eigen vector
// 		Eigen::VectorXd odom_tf_goal_quaternion_worker(4);
// 		odom_tf_goal_quaternion_worker << tfQuat_initial.x(), tfQuat_initial.y(), tfQuat_initial.z(), tfQuat_initial.w();
// 		// Convert the Eigen vector to an Eigen quaternion
// 		Eigen::Quaterniond odom_tf_goal_quaternion(odom_tf_goal_quaternion_worker(3),odom_tf_goal_quaternion_worker(0),odom_tf_goal_quaternion_worker(1),odom_tf_goal_quaternion_worker(2));
// 		Eigen::Quaterniond odom_tf_goal_quaternion_normalized;
// 		odom_tf_goal_quaternion_normalized = odom_tf_goal_quaternion.normalized();
// 		odom_tf_goal_rotation_matrix = odom_tf_goal_quaternion_normalized.toRotationMatrix();
// 		odom_tf_goal_homogeneous_matrix << odom_tf_goal_rotation_matrix, odom_tf_goal_translation, 0, 0, 0, 1;
// 		// Use the transform to convert the received goal to a global goal
// 		global_goal_normal = odom_tf_goal_rotation_matrix * goal_normal;
// 		Eigen::VectorXd goal_normal_homogeneous_worker(4);
// 		goal_normal_homogeneous_worker << goal_normal(0)*goal_offset, goal_normal(1)*goal_offset, goal_normal(2)*goal_offset, 1;
// 		Eigen::VectorXd homogeneous_global_goal(4);
// 		Eigen::VectorXd homogeneous_goal_worker(4);
// 		homogeneous_goal_worker = homogeneous_goal - goal_normal_homogeneous_worker;
// 		homogeneous_goal_worker(3) = 1.0;
// 		homogeneous_global_goal = odom_tf_goal_homogeneous_matrix * (homogeneous_goal_worker);
// 		global_goal << homogeneous_global_goal(0), homogeneous_global_goal(1), homogeneous_global_goal(2);
// 		// Update the relative goal based on the robot's global position
// 		update_rel_goal();
// 		// Plan RRT Connect Path and send it for execution
// 		// Set move group planning constraints
// 		move_group_ptr->setNumPlanningAttempts(num_attempts_initial_goal);
// 		move_group_ptr->setPlanningTime(moveit_planning_time_initial_goal);
// 		// Reset tolerances
// 		goal_tolerance_pose = goal_tolerance_pose_default;
// 		goal_tolerance_angle = goal_tolerance_angle_default;
// 		// Set kinematic constraints
// 		moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(end_effector_name, goal_pose, goal_tolerance_pose, goal_tolerance_angle);
// 		std::cout<<"Added kinematic constraints"<<std::endl;
// 		ROS_INFO_STREAM(pose_goal);
// 		// planning_scene_monitor::LockedPlanningSceneRO lscene(*psmPtr);
// 		// std::cout<<"Locked planning scene monitor"<<std::endl;
// 		std::cout<<"Calling planning pipeline to generate plan"<<std::endl;
// 		/* Now, call the pipeline and check whether planning was successful. */
// 		/* Check that the planning was successful */
// 		moveit_msgs::MotionPlanResponse response;
// 		response_ptr = &response;
// 		while (true)
// 		{
// 			std::cout<<"Adding pose goal"<<std::endl;
// 			moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(end_effector_name, goal_pose, goal_tolerance_pose, goal_tolerance_angle);
// 			plan_req.group_name = PLANNING_GROUP;
// 			plan_req.goal_constraints.clear();
// 			plan_req.goal_constraints.push_back(pose_goal);
// 			// Update current state to be the last planned state
// 			std::cout<<"Updating start state"<<std::endl;
// 			plan_req.start_state.joint_state.name = prev_plan_res.trajectory.joint_trajectory.joint_names;
// 			plan_req.start_state.joint_state.position = prev_plan_res.trajectory.joint_trajectory.points[prev_plan_res.trajectory.joint_trajectory.points.size() - 1].positions;
// 			plan_req.start_state.joint_state.velocity = prev_plan_res.trajectory.joint_trajectory.points[prev_plan_res.trajectory.joint_trajectory.points.size() - 1].velocities;
// 			plan_req.start_state.joint_state.effort = prev_plan_res.trajectory.joint_trajectory.points[prev_plan_res.trajectory.joint_trajectory.points.size() - 1].effort;
// 			// // Lock the visual planner
// 			// planning_scene_monitor::LockedPlanningSceneRO lscene(*psmPtr);
// 			std::cout<<"Generating plan"<<std::endl;
// 			planning_pipeline_global_ptr->generatePlan(*psmPtr, plan_req, plan_res);
// 			if ( sqrt(pow(goal_tolerance_pose[0],2)+pow(goal_tolerance_pose[1],2)+pow(goal_tolerance_pose[2],2)) > goal_tolerance_pose_adjusted_threshold)
// 			{
// 				///////// REMOVE or change functionality to work with Yuqing's goal node ///////////
// 				if (num_attempts < num_max_attempts)
// 				{
// 					std::cout<<"Couldn't sufficiently plan for point, grabbing new goal"<<std::endl;
// 					state = 1;
// 					num_attempts += 1;
// 					return;
// 				}
// 				state = 0;
// 				ROS_INFO("Couldn't find a suitable path, returning to waiting");
// 				// Let the main_state_machine node know that the robot is ready
// 				status.data = 3;
// 				state_input_pub_ptr->publish(status);
// 				num_attempts = 0;
// 				return;
// 			}
// 			if (plan_res.error_code_.val != plan_res.error_code_.SUCCESS)
// 			{
// 				ROS_ERROR("Could not compute plan successfully, increasing tolerances");
// 				// Increment goal tolerance and break out if tolerances are too large
// 				goal_tolerance_pose[0] = goal_tolerance_pose[0] + goal_tolerance_pose_adjustment;
// 				goal_tolerance_pose[1] = goal_tolerance_pose[1] + goal_tolerance_pose_adjustment;
// 				goal_tolerance_pose[2] = goal_tolerance_pose[2] + goal_tolerance_pose_adjustment;
// 				continue;
// 			}
// 			plan_res.getMessage(response);
// 			// moveit_msgs::RobotTrajectory trajectory_worker;
// 			// trajectory_worker = response.trajectory;
// 			// trajectory_worker.joint_trajectory = response.trajectory.joint_trajectory;
// 			if((*psmPtr)->isPathValid(plan_req.start_state, response.trajectory, PLANNING_GROUP))
// 			{
// 				break;
// 			}
// 			else
// 			{
// 				continue;
// 			}
// 		}
// 		num_attempts = 0;
// 		/////////////// Visualize the result
// 		moveit_msgs::DisplayTrajectory display_trajectory;
// 		/* Visualize the trajectory */
// 		ROS_INFO("Visualizing the trajectory");
// 		display_trajectory.trajectory_start = response.trajectory_start;
// 		display_trajectory.trajectory.clear();
// 		display_trajectory.trajectory.push_back(response.trajectory);
// 		display_publisher_ptr->publish(display_trajectory);
// 		visual_tools_ptr->publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
// 		visual_tools_ptr->trigger();
// 		///////////////
// 		// Set the trajectory and execute the plan
// 		my_plan.trajectory_ = response.trajectory;
// 		// moveitSuccess = move_group_ptr->plan(my_plan);
// 		std::cout<<"my_plan.planning_time_ is: "<<my_plan.planning_time_<<std::endl;
// 		std::cout<<"my_plan.start_state_ is: "<<my_plan.start_state_<<std::endl;
// 		std::cout<<"my_plan.trajectory_ is: "<<my_plan.trajectory_<<std::endl;
// 		std::cout<<"plan_res.planning_time_ is: "<<plan_res.planning_time_<<std::endl;
// 		// moveit_plans_pub_ptr->publish(my_plan);
// 		move_group_ptr->asyncExecute(my_plan);
// 		// ros::param::set("/tf_moveit_goalsetNode/manipulation_state", "standby");
// 		prev_plan_res = response;
// 		plan_start = ros::Time::now();
// 		state = 2;
// 		status.data = 2;
// 		state_input_pub_ptr->publish(status);
// 		ROS_INFO("Moving to target");
// 	}
// }
// State machine commands callback
void state_output_callback(const std_msgs::Int32::ConstPtr& msg)
{
	// Functions:
	// 1 = e_stop > Shut off power to motors
	// 2 = hold > Move to and hold plate
	// 3 = compact > Return to home position
	// # = caution > Motors stop moving, but maintain with lower torque threshold

	// Statuses:
	// 1 = initializing > Command received, but not executing yet(e.g.detecting hands)[ROS INFO]
	// 2 = executing > Command being executed(e.g.moving to target)
	// 3 = waiting > Command completed / performing holding task, ready for next command(maintaining position in 3d space)

	if (msg->data == 2)
	{
		if (state == 0)
		{
			state = 1;
			status.data = 1;
			state_input_pub_ptr->publish(status);
			ROS_INFO("Planning to move to target");
		}
	}

	if (msg->data == 3)
	{
		if (state == 4)
		{
			state = 5;
			ros::param::set("/tf_moveit_goalsetNode/manipulation_state", "velocity");
			status.data = 1;
			state_input_pub_ptr->publish(status);
			// // Update the normal vector
			// update_rel_goal();
			// // Set the desired end-effector velocity
			// desired_velocity << -goal_normal * naive_push_speed, 0, 0, 0;
			// // Grab the starting end-effector pose
			// naive_pull_starting_pose = (*robot_state_ptr)->getGlobalLinkTransform(end_effector_name);
			// Let the main_state_machine node know that the robot is executing
			ROS_INFO("Leaving target");
			status.data = 2;
			state_input_pub_ptr->publish(status);
		}
	}
}
// MoveIt callback for trajectory finishes
void execute_trajectory_feedback_callback(const moveit_msgs::MoveGroupActionFeedback::ConstPtr& msg)
{
	std::cout<<"execute_trajectory_feedback_callback called"<<std::endl;
	std::cout<<"msg->feedback.state is: "<<msg->feedback.state<<std::endl;
	std::cout<<"msg->status.text is: "<<msg->status.text<<std::endl;
	// Check if the trajectory has finished
	if (msg->feedback.state == "IDLE" and msg->status.text == "Solution was found and executed.")
	{
		// Check if any time has passed
		// if (ros::Time::now() - plan_start > ros::Duration(plan_execution_start_delay))
		if(true)
		{
			std::cout<<"row::Time::now() is: "<<ros::Time::now()<<std::endl;
			std::cout<<"plan_start is: "<<plan_start<<std::endl;
			// Check if the robot was moving to a target
			if (state == 2)
			{
				state = 3;
				ROS_INFO("Successfully reached target offset; extending");
				ros::param::set("/tf_moveit_goalsetNode/manipulation_state", "velocity");
				// Update the normal vector
				// update_rel_goal();
				// // Set the desired end-effector velocity
				// desired_velocity << goal_normal * naive_push_speed, 0, 0, 0;
			}
			// Check if the robot was returning to home
			else if (state == 6)
			{
				state = 0;
				ROS_INFO("Successfully returned to home; waiting");
				// Let the main_state_machine node know that the robot is ready
				status.data = 3;
				state_input_pub_ptr->publish(status);
			}
		}
	}
}
// HEBI joint angles callback
void hebiJointsCallback(const sensor_msgs::JointState::ConstPtr & hebimsg)
{
	// Grab the joint angles and velocities of the robot
	hebiJointAngles.at(0) = hebimsg->position[0];
	hebiJointAngles.at(1) = hebimsg->position[1];
	hebiJointAngles.at(2) = hebimsg->position[2];
	hebiJointAngles.at(3) = hebimsg->position[3];
	hebiJointAngVelocities.at(0) = hebimsg->velocity[0];
	hebiJointAngVelocities.at(1) = hebimsg->velocity[1];
	hebiJointAngVelocities.at(2) = hebimsg->velocity[2];
	hebiJointAngVelocities.at(3) = hebimsg->velocity[3];
}

int main(int argc, char** argv)
{	
    // Initialize ROS
	ros::init(argc, argv, "KDC_actuated_manipulation_node");

	// Construct node
	ros::NodeHandle node_handle;
	ros::NodeHandle* node_handle_ptr;
	node_handle_ptr = &node_handle;

	ros::Duration(10).sleep();

	ros::AsyncSpinner spinner(0);
	spinner.start();

    // ROS-dependent initializations
	// most_recent_transform = ros::Time::now();
	// Global variable pointer attachments
    // tf::TransformListener listener;
    // listener_ptr = &listener;
	
	//code stolen from hebi_cpp_api_examples/src/basic/group_node.cpp
	// connect to HEBI joints on network through UDP connection
    for (int num_tries = 0; num_tries < 3; num_tries++) {
        hebi::Lookup lookup;
        group = lookup.getGroupFromNames(families, names);
        if (group) {
            //print hebi modules to terminal
            auto entry_list = lookup.getEntryList();

            for (size_t hebi_iter = 0; hebi_iter < entry_list->size(); ++hebi_iter)
            {
                std::cout << (*entry_list)[hebi_iter].family_ << " | " << (*entry_list)[hebi_iter].name_ << std::endl;
            }
            break;
        }
        ROS_WARN("Could not find group actuators, trying again...");
        ros::Duration(1.0).sleep();
    }
	int group_size;
	if (group){
		group_size = group->size();
	} else {
		ROS_WARN("Could not find group actuators, setting dummy size of 3");
	    group_size = 3;
	}

	// Initialize Local Naive Push Variables
	Eigen::VectorXd joint_velocities(group_size);
	hebi::GroupCommand groupCommand(group_size);
	hebi::GroupFeedback group_feedback(group_size);
	Eigen::VectorXd motor_torques(group_size);
    
	// Set initial matrix values
	joint_velocities.setZero();
	desired_velocity.setZero();
	impedance_goal.setZero();
	impedance_global_goal.setZero();
	desired_cartesian_forces.setZero();
	impedance_position_gain << 50, 0, 0; // X is the normal direction
	impedance_derivative_gain << 2, 2, 2; // X is the normal direction

	// MoveIt initializations and declarations
	//////////////////////////////////////////////////////////////
	// MoveIt Motion Planning Pipeline Tutorial Initializations
	// Load Robot Model
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	move_group_ptr = &move_group;
	std::cout<<"Move_group initialized"<<std::endl;
	// Adjust MoveIt Planner settings
	move_group.setPlannerId(move_group_planner_id);
	std::cout<<"Move_group planner set"<<std::endl;
	// Load robot model
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
	std::cout<<"Robot model loaded"<<std::endl;
	// Create Planning Scene
	planning_scene::PlanningScenePtr psm(new planning_scene::PlanningScene(robot_model));
	psmPtr = &psm;
	/* listen for planning scene messages on topic /XXX and apply them to the internal planning scene
						the internal pla0.018115nning scene accordingly */
	// psm->startSceneMonitor();
	/* listens to changes of world geometry, collision objects, and (optionally) octomaps
								world geometry, collision objects and optionally octomaps */
	// psm->startWorldGeometryMonitor();
	/* listen to joint state updates as well as changes in attached collision objects
						and update the internal planning scene accordingly*/
	// psm->startStateMonitor();
	std::cout<<"Planning scene initialized"<<std::endl;

	/* We can also use the RobotModelLoader to get a robot model which contains the robot's kinematic information */
	/////////////////////////////////////////////////
	/* We can get the most up to date robot state from the PlanningSceneMonitor by locking the internal planning scene
	for reading. This lock ensures that the underlying scene isn't updated while we are reading it's state.
	RobotState's are useful for computing the forward and inverse kinematics of the robot among many other uses */
	moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
	robot_state_ptr = &robot_state;
	std::cout<<"Robot state set up"<<std::endl;
	/* Create a JointModelGroup to keep track of the current robot pose and planning group. The Joint Model
	group is useful for dealing with one set of joints at a time such as a left arm or a end effector */
	joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
	// Create Planning Pipeline
	planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, *node_handle_ptr, "/KDC_actuated_test_copy/planning_plugin", "KDC_actuated_test_copy/request_adapters"));
	planning_pipeline_global_ptr = planning_pipeline;
	std::cout<<"Planning pipeline initialized"<<std::endl;
	// Set Up Visualization Tools
	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("world");
	visual_tools_ptr = &visual_tools;
	visual_tools.deleteAllMarkers();

	/* Remote control is an introspection tool that allows users to step through a high level script
	via buttons and keyboard shortcuts in RViz */
	visual_tools.loadRemoteControl();

	/* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
	Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
	text_pose.translation().z() = 1.75;
	visual_tools.publishText(text_pose, "Motion Planning Pipeline Demo", rvt::WHITE, rvt::XLARGE);

	/* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
	visual_tools.trigger();

	/* We can also use visual_tools to wait for user input */
	// visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
	std::cout<<"Visual tools initialized"<<std::endl;
	//////////////////////////////////////////////////////////////

	// Initialize Publishers
	ros::Publisher state_input_pub = node_handle.advertise<std_msgs::Int32>("/state_input", 5);
	state_input_pub_ptr = &state_input_pub;
	ros::Publisher desired_efforts_pub = node_handle.advertise<sensor_msgs::JointState>("/desired_hebi_efforts", 1);
	ros::Publisher display_publisher = node_handle_ptr->advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	display_publisher_ptr = &display_publisher;
	// ros::Publisher moveit_plans_pub = node_handle.advertise<moveit_msgs::MotionPlanRequest>("/moveit_plans", 1);
	// moveit_plans_pub_ptr = &moveit_plans_pub;
	ros::Duration(0.5).sleep();

	// Publish initial state (waiting)
	status.data = 3;
	state_input_pub.publish(status);

	// Initialize Subscribers
	// ros::Subscriber camera_goal_sub = node_handle.subscribe("/goal_cam2", 1, camera_goal_callback);
	ros::Subscriber state_output_sub = node_handle.subscribe("/state_output", 1, state_output_callback);
	ros::Subscriber execute_action_sub = node_handle.subscribe("/execute_trajectory/feedback", 5, execute_trajectory_feedback_callback);
	ros::Subscriber hebi_joints_sub = node_handle.subscribe("/hebi_joints", 1, hebiJointsCallback);
	ros::Subscriber goal_sub = node_handle.subscribe("/goal", 1, goal_callback);

	std::cout<<"Publishers and subscribers initialized"<<std::endl;

	std::cout<<"Looping and ready"<<std::endl;
	while (ros::ok())
	{
        // Check if the robot has been commanded to move and has received an updated local goal
        if (state == -1)
        {
            // Plan RRT Connect Path and send it for execution
            // Set move group planning constraints
            move_group_ptr->setNumPlanningAttempts(num_attempts_initial_goal);
            move_group_ptr->setPlanningTime(moveit_planning_time_initial_goal);
            // Reset tolerances
            goal_tolerance_pose = goal_tolerance_pose_default;
            goal_tolerance_angle = goal_tolerance_angle_default;
            // planning_scene_monitor::LockedPlanningSceneRO lscene(*psmPtr);
            // std::cout<<"Locked planning scene monitor"<<std::endl;
            std::cout<<"Calling planning pipeline to generate plan"<<std::endl;
            /* Now, call the pipeline and check whether planning was successful. */
            /* Check that the planning was successful */
            moveit_msgs::MotionPlanResponse response;
            response_ptr = &response;
            while (true)
            {
                std::cout<<"Adding pose goal"<<std::endl;
                moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(end_effector_name, goal_pose, goal_tolerance_pose, goal_tolerance_angle);
                plan_req.group_name = PLANNING_GROUP;
                plan_req.goal_constraints.clear();
                plan_req.goal_constraints.push_back(pose_goal);
                // Update current state to be the last planned state
                std::cout<<"Updating start state"<<std::endl;
                plan_req.start_state.joint_state.name = prev_plan_res.trajectory.joint_trajectory.joint_names;
                plan_req.start_state.joint_state.position = prev_plan_res.trajectory.joint_trajectory.points[prev_plan_res.trajectory.joint_trajectory.points.size() - 1].positions;
                plan_req.start_state.joint_state.velocity = prev_plan_res.trajectory.joint_trajectory.points[prev_plan_res.trajectory.joint_trajectory.points.size() - 1].velocities;
                plan_req.start_state.joint_state.effort = prev_plan_res.trajectory.joint_trajectory.points[prev_plan_res.trajectory.joint_trajectory.points.size() - 1].effort;
                // // Lock the visual planner
                // planning_scene_monitor::LockedPlanningSceneRO lscene(*psmPtr);
                std::cout<<"Generating plan"<<std::endl;
                planning_pipeline_global_ptr->generatePlan(*psmPtr, plan_req, plan_res);
                if ( sqrt(pow(goal_tolerance_pose[0],2)+pow(goal_tolerance_pose[1],2)+pow(goal_tolerance_pose[2],2)) > goal_tolerance_pose_adjusted_threshold)
                {
                    if (num_attempts < num_max_attempts)
                    {
                        std::cout<<"Couldn't sufficiently plan for point, trying again"<<std::endl;
                        num_attempts += 1;
                        break;
                    }
                    state = 0;
                    ROS_INFO("Couldn't find a suitable path, returning to waiting");
                    // Let the main_state_machine node know that the robot is ready
                    status.data = 3;
                    state_input_pub_ptr->publish(status);
                    num_attempts = 0;
                    break;
                }
                if (plan_res.error_code_.val != plan_res.error_code_.SUCCESS)
                {
                    ROS_ERROR("Could not compute plan successfully, increasing tolerances");
                    // Increment goal tolerance and break out if tolerances are too large
                    goal_tolerance_pose[0] = goal_tolerance_pose[0] + goal_tolerance_pose_adjustment;
                    goal_tolerance_pose[1] = goal_tolerance_pose[1] + goal_tolerance_pose_adjustment;
                    goal_tolerance_pose[2] = goal_tolerance_pose[2] + goal_tolerance_pose_adjustment;
                    continue;
                }
                plan_res.getMessage(response);
                // moveit_msgs::RobotTrajectory trajectory_worker;
                // trajectory_worker = response.trajectory;
                // trajectory_worker.joint_trajectory = response.trajectory.joint_trajectory;
                if((*psmPtr)->isPathValid(plan_req.start_state, response.trajectory, PLANNING_GROUP))
                {
                    break;
                }
                else
                {
                    continue;
                }
            }
            num_attempts = 0;
            // Continue through outer while loop
            if (plan_res.error_code_.val != plan_res.error_code_.SUCCESS)
            {
                continue;
            }
            /////////////// Visualize the result
            moveit_msgs::DisplayTrajectory display_trajectory;
            /* Visualize the trajectory */
            ROS_INFO("Visualizing the trajectory");
            display_trajectory.trajectory_start = response.trajectory_start;
            display_trajectory.trajectory.clear();
            display_trajectory.trajectory.push_back(response.trajectory);
            display_publisher_ptr->publish(display_trajectory);
            visual_tools_ptr->publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
            visual_tools_ptr->trigger();
            ///////////////
            // Set the trajectory and execute the plan
            my_plan.trajectory_ = response.trajectory;
            // moveitSuccess = move_group_ptr->plan(my_plan);
            std::cout<<"my_plan.planning_time_ is: "<<my_plan.planning_time_<<std::endl;
            std::cout<<"my_plan.start_state_ is: "<<my_plan.start_state_<<std::endl;
            std::cout<<"my_plan.trajectory_ is: "<<my_plan.trajectory_<<std::endl;
            std::cout<<"plan_res.planning_time_ is: "<<plan_res.planning_time_<<std::endl;
            // moveit_plans_pub_ptr->publish(my_plan);
            move_group_ptr->execute(my_plan);
            // ros::param::set("/tf_moveit_goalsetNode/manipulation_state", "standby");
            prev_plan_res = response;
            plan_start = ros::Time::now();
            state = 2;
            status.data = 2;
            state_input_pub_ptr->publish(status);
            ROS_INFO("Moving to target");
        }
		// Check if the robot is executing trajectories to target
		// if (state == 2)
		if (state == 69)
		{
			stitching_loop_number += 1;
			// prev_plan_res contains the current, time-stamped RRT plan
			// Determine how far into the future of the current plan to begin the next plan
			std::cout<<"Beginning new planning trajectory (state == 2)"<<std::endl;
			std::cout<<"ros::Time::now().toSec() is: "<<ros::Time::now().toSec()<<std::endl;
			std::cout<<"plan_start.toSec() is: "<<plan_start.toSec()<<std::endl;
			std::cout<<"prev_plan_res.trajectory.joint_trajectory.points.size() is: "<<prev_plan_res.trajectory.joint_trajectory.points.size()<<std::endl;
			desired_plan_start_time = planning_time_offset + stitching_time_offset + ros::Time::now().toSec() - plan_start.toSec();
			// Loop through the previously planned path until you find the point in the path corresponding to the desired time
			trajectory_start_point_success = 0;
			std::cout<<"Searching old plan for breakaway point"<<std::endl;
			for (unsigned int i = 0; i < (prev_plan_res.trajectory.joint_trajectory.points.size()); ++i)
			{
				std::cout<<"The time_from_start.toSec() of point "<<i<<" is: "<<prev_plan_res.trajectory.joint_trajectory.points[i].time_from_start.toSec()<<std::endl; /////////////////////////
				if (prev_plan_res.trajectory.joint_trajectory.points[i].time_from_start.toSec() > desired_plan_start_time)
				{
					// Set the start of the plan to be equal to the corresponding point
					trajectory_start_point = i;
					trajectory_start_point_success = 1;
					// Fill in the information for the new starting RobotState
					plan_req.start_state.joint_state.header.stamp = ros::Time::now();
					plan_req.start_state.joint_state.header.frame_id = goal_pose.header.frame_id;
					plan_req.start_state.joint_state.name = prev_plan_res.trajectory.joint_trajectory.joint_names;
					plan_req.start_state.joint_state.position = prev_plan_res.trajectory.joint_trajectory.points[i].positions;
					plan_req.start_state.joint_state.velocity = prev_plan_res.trajectory.joint_trajectory.points[i].velocities;
					plan_req.start_state.joint_state.effort = prev_plan_res.trajectory.joint_trajectory.points[i].effort;
					std::cout<<"Breakaway point found"<<std::endl;
					// Stop looping
					break;
				}
			}
			// If desired_plan_start_time is greater than the time to finish the path, don't bother planning a new one
			if (!trajectory_start_point_success)
			{
				std::cout<<"No sufficient breakaway point found, looping"<<std::endl;
				continue;
			}
			// Use RRT to plan a path from the beginning point just extracted to the current goal point, updated based on the t_265 localization
			// Plan RRT Connect Path and send it for execution
			// Set move group planning constraints
			move_group.setNumPlanningAttempts(num_attempts_stitching);
			move_group.setPlanningTime(planning_time_offset);
			// Set kinematic constraints
			std::cout<<"Updating current goal"<<std::endl;
			moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(end_effector_name, goal_pose, goal_tolerance_pose, goal_tolerance_angle);
			plan_req.group_name = PLANNING_GROUP;
			plan_req.goal_constraints.clear();
			plan_req.goal_constraints.push_back(pose_goal);
			// Lock the visual planner
			{
				// planning_scene_monitor::LockedPlanningSceneRO lscene(*psmPtr);
				/* Now, call the pipeline and check whether planning was successful. */
				std::cout<<"Generating new plan"<<std::endl;
				planning_pipeline->generatePlan(*psmPtr, plan_req, plan_res);
			}
			/* Now, call the pipeline and check whether planning was successful. */
			/* Check that the planning was successful */
			if (plan_res.error_code_.val != plan_res.error_code_.SUCCESS)
			{
				ROS_ERROR("Could not compute plan successfully");
				continue;
				// TODO: CONSIDER ADDING IN THE TOLERANCE INCREASES
			}
			std::cout<<"Plan generated successfully"<<std::endl;
			// Stitch the new response (response.trajectory.joint_trajectory.points) onto the beginning of the old one
			stitch_plan_start = ros::Time::now();
			// Store plan
			moveit_msgs::MotionPlanResponse response;
			response_ptr = &response;
			plan_res.getMessage(response);
			// Record the memory size of the arrays
			// int new_trajectory_length = response_ptr->trajectory.joint_trajectory.points.size();
			int new_trajectory_length = response_ptr->trajectory.joint_trajectory.points.size();
			// Create a working array to hold the stitched trajectory
			std::cout<<"trajectory_start_point is: "<<trajectory_start_point<<std::endl;
			std::cout<<"new_trajectory_length is: "<<new_trajectory_length<<std::endl;
			// std::cout<<"response_ptr->trajectory.joint_trajectory is: "<<response_ptr->trajectory.joint_trajectory<<std::endl;
			// std::vector<trajectory_msgs::JointTrajectoryPoint> working_trajectory_array(trajectory_start_point + new_trajectory_length);
			// int const working_length = trajectory_start_point + new_trajectory_length;
			// std::array<trajectory_msgs::JointTrajectoryPoint, working_length> working_trajectory_array;
			//std::cout << prev_plan_res.trajectory.joint_trajectory.points.begin();
			std::cout<<"Copying trajectories"<<std::endl;
			trajectory_msgs::JointTrajectoryPoint working_trajectory_array [1 + trajectory_start_point + new_trajectory_length];
			try
			{
				std::cout<<"First copy"<<std::endl;
				std::cout<<"prev_plan_res.trajectory.joint_trajectory is: "<<prev_plan_res.trajectory.joint_trajectory<<std::endl;
				std::cout<<"trajectory_start_point is: "<<trajectory_start_point<<std::endl;
				std::cout<<"stitching_loop_number is: "<<stitching_loop_number<<std::endl;
				std::cout<<"trajectory_start_point is: "<<trajectory_start_point<<std::endl;
				std::cout<<"new_trajectory_length is: "<<new_trajectory_length<<std::endl;
				// std::cout<<"working_trajectory_array.size() is: "<<working_trajectory_array.size()<<std::endl;
				// std::copy(response_ptr->trajectory.joint_trajectory.points, response_ptr->trajectory.joint_trajectory.points + trajectory_start_point, working_trajectory_array);
				// std::copy(response_ptr->trajectory.joint_trajectory.points, response_ptr->trajectory.joint_trajectory.points + new_trajectory_length, working_trajectory_array + trajectory_start_point + 1);
				// std::copy(response_ptr->trajectory.joint_trajectory.points.begin(), response_ptr->trajectory.joint_trajectory.points.begin() + trajectory_start_point, working_trajectory_array.begin());
				// trajectory_msgs::JointTrajectoryPoint working_trajectory_array[];
				for (unsigned int ii = 0; ii<= trajectory_start_point; ii++)
				{
					std::cout<<"First copy ii: "<<ii<<std::endl;
					working_trajectory_array[ii] = prev_plan_res.trajectory.joint_trajectory.points[ii];
				}
				std::cout<<"Second copy"<<std::endl;
				for (unsigned int ii = 0; ii < new_trajectory_length; ii++)
				{
					std::cout<<"Second copy ii: "<<ii<<std::endl;
					working_trajectory_array[trajectory_start_point + ii + 1] = response.trajectory.joint_trajectory.points[ii];
				}
				// std::copy(response_ptr->trajectory.joint_trajectory.points.begin(), response_ptr->trajectory.joint_trajectory.points.begin() + new_trajectory_length, working_trajectory_array.begin() + trajectory_start_point + 1);
				// std::copy(response_ptr->trajectory.joint_trajectory.points.begin(), response_ptr->trajectory.joint_trajectory.points.back(), working_trajectory_array.begin() + trajectory_start_point + 1);
			}
			catch(...)
			{
				std::cout<<"Copying trajectories failed"<<std::endl;
				std::cout<<"prev_plan_res.trajectory.joint_trajectory.points[0] is: "<<prev_plan_res.trajectory.joint_trajectory.points[0]<<std::endl;
				std::cout<<"trajectory_start_point is: "<<trajectory_start_point<<std::endl;
				std::cout<<"response_ptr->trajectory.joint_trajectory.points[0] is: "<<response_ptr->trajectory.joint_trajectory.points[0]<<std::endl;
				std::cout<<"new_trajectory_length is: "<<new_trajectory_length<<std::endl;
				exit(0);
			}
			// Loop through the full working trajectory and find the new starting point (updated for how far the robot has traversed)
			// Update the time stamps accordingly
			std::cout<<"Finding new starting point and updating time stamps"<<std::endl;
			new_plan_origin_found = 0;
			////////////// // Naive Method
			ros::Duration trajectory_start_time;
			//////////////
			for (unsigned int j = 0; j < (trajectory_start_point + new_trajectory_length); ++j)
			{
				// Check if the planning has taken so long that the manipulator has already reached the new trajectory, stop trying to fix the trajectory and restart
				if ((j > trajectory_start_point) && (new_plan_origin_found == 0))
				{
					std::cout<<"Planning time insufficient, increasing"<<std::endl;
					// If there wasn't enough time to plan the trajectory, increase the planning time to the total time that it took to plan + the increase rate
					std::cout<<"ros::Time::now() is: "<<ros::Time::now()<<std::endl;
					std::cout<<"plan_start is: "<<plan_start<<std::endl;
					std::cout<<"(ros::Time::now() - plan_start).toSec() is: "<<(ros::Time::now() - plan_start).toSec()<<std::endl;
					std::cout<<"(ros::Time::now() - plan_start).toSec() * (1.0 + planning_time_offset_increase_rate) is: "<<(ros::Time::now() - plan_start).toSec() * (1.0 + planning_time_offset_increase_rate)<<std::endl;
					planning_time_offset = (ros::Time::now() - plan_start).toSec() * (1.0 + planning_time_offset_increase_rate);
					ROS_INFO("Failed to plan in time, new planning_time_offset:%s", planning_time_offset);
					break;
				}
				// If a starting point in the old trajectory has been found, set it as the new starting point for the final, stitched and cut trajectory
				if ((new_plan_origin_found == 0) && (working_trajectory_array[j].time_from_start.toSec() > ((ros::Time::now() - plan_start).toSec() + stitching_time_offset)))
				{
					std::cout<<"New starting point found, adjusting trajectory time stamps to match"<<std::endl;
					// Record the cut point
					new_plan_origin = j;
					new_plan_origin_found = 1;
					// Record the time difference
					//////////// // Naive Method
					trajectory_start_time = working_trajectory_array[j].time_from_start;
					////////////
					stitching_plan_time_diff = -working_trajectory_array[j].time_from_start;
					working_trajectory_array[j].time_from_start = ros::Duration(0.0);
					//////////
					// stitching_plan_time_diff = ros::Time::now() - plan_start;
					// working_trajectory_array[j].time_from_start = working_trajectory_array[j] - stitching_plan_time_diff;
					//////////
					// Update the response start state
					response.trajectory_start.joint_state.position = working_trajectory_array[j].positions;
					response.trajectory_start.joint_state.velocity = working_trajectory_array[j].velocities;
					response.trajectory_start.joint_state.effort = working_trajectory_array[j].effort;
				}
				// If a starting point was found in a previous loop, update the time from start of this point
				else if (new_plan_origin_found == 1)
				{
					// Subtract the time already elapsed to get the new time from start
					working_trajectory_array[j].time_from_start = working_trajectory_array[j].time_from_start + ros::Duration(stitching_plan_time_diff);
					//////////
					// working_trajectory_array[j].time_from_start = working_trajectory_array[j].time_from_start - stitching_plan_time_diff);
					//////////
					// Check to see if we're at the pivot point between the old and new trajectories
					if (j == trajectory_start_point)
					{
						// Because the new trajectory starts off with a time of 0, the duration to be added needs to be set to the current duration plus one step
						stitching_plan_time_diff = working_trajectory_array[j].time_from_start + working_trajectory_array[j].time_from_start - working_trajectory_array[j-1].time_from_start;
					}
				}
			}
			// Check if the plan was successfully stitched together
			if (new_plan_origin_found)
			{
				std::cout<<"Plan successfully stitched"<<std::endl;
				// Update the trajectory
				trajectory_msgs::JointTrajectoryPoint response_worker [trajectory_start_point + new_trajectory_length - new_plan_origin];
				for (unsigned int ii = new_plan_origin; ii < trajectory_start_point + new_trajectory_length; ii++)
				{
					response_worker[ii - new_plan_origin] = working_trajectory_array[ii];
				}
				// response.trajectory.joint_trajectory.points = working_trajectory_array[new_plan_origin:-1];

				// std::copy(working_trajectory_array.begin()+new_plan_origin, working_trajectory_array.end(), response_ptr->trajectory.joint_trajectory.points.begin());
				
				/////////////// Visualize the result
				//ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
				moveit_msgs::DisplayTrajectory display_trajectory;

				/* Visualize the trajectory */
				ROS_INFO("Visualizing the new trajectory");

				display_trajectory.trajectory_start = response.trajectory_start;
				display_trajectory.trajectory.clear();
				display_trajectory.trajectory.push_back(response.trajectory);
				display_publisher.publish(display_trajectory);
				visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
				visual_tools.trigger();
				///////////////
				// Update and Execute the plan
				std::cout<<"Executing new trajectory"<<std::endl;
				my_plan.trajectory_ = response.trajectory;
				// my_plan.start_state_ = response.trajectory_start;
				//////////////////// // Naive Method
				ros::console::shutdown();
				std::cout.clear();
				std::cout<<"/////////////////////////////////"<<std::endl;
				std::cout<<"stitching_loop_number is: "<<stitching_loop_number<<std::endl;
				std::cout<<"plan_start is: "<<plan_start<<std::endl;
				std::cout<<"ros::Time::now() is: "<<ros::Time::now()<<std::endl;
				std::cout<<"trajectory_start_time is: "<<trajectory_start_time<<std::endl;
				std::cout<<"plan_start - ros::Time::now() + trajectory_start_time is: "<<(plan_start - ros::Time::now() + trajectory_start_time)<<std::endl;
				std::cout.setstate(std::ios_base::failbit);
				ros::Duration(plan_start - ros::Time::now() + trajectory_start_time - ros::Duration(0.03)).sleep();
				////////////////////
				// moveitSuccess = move_group.plan(my_plan);
				// moveit_plans_pub_ptr->publish(my_plan);
				move_group.stop();
				move_group.asyncExecute(my_plan);
				prev_plan_res = response;
				ROS_INFO("Plan successfully executing.  Time to plan: %s", (ros::Time::now() - plan_start).toSec());
				plan_start = ros::Time::now();
			}
			else
			{
				// If there wasn't enough time to stitch the trajectories together, increase the stitching time to the total time that it took to stitch + the increase rate
				std::cout<<"Stitching time insufficient, increasing"<<std::endl;
				stitching_time_offset = ((ros::Time::now() - plan_start).toSec() - planning_time_offset) * (1.0 + stitching_time_offset_increase_rate);
				ROS_INFO("Failed to stitch in time, new stitching_time_offset:%s", stitching_time_offset);
			}
			new_plan_origin_found = 0;
		}
		// Check if the robot is naively moving toward the target (admittance control)
		if (state == 3)
		{
			// // Update the Jacobian
			// robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, wristJacobian);
			// // Get the joint forces
			// if (group->getNextFeedback(group_feedback))
			// {
			// 	motor_torques = group_feedback.getEffort();
			// }
			// // Update the end-effector force
			// end_effector_force = wristJacobian * motor_torques;
			// // Check to see if the force on the end-effector is below the threshold
			// if (sqrt(pow(end_effector_force(0),2)+pow(end_effector_force(1),2)+pow(end_effector_force(2),2)) < naive_push_force_threshold)
			// {
			// 	// Update the joint velocities
			// 	joint_velocities = wristJacobian.inverse() * desired_velocity;
			// 	groupCommand.setVelocity(joint_velocities);
			// 	group->sendCommand(groupCommand);
			// }
			// // If it exceeds the threshold, set up impedance control
			// else
			// {
			// 	// Stop the end-effector
			// 	joint_velocities.setZero();
			// 	groupCommand.setVelocity(joint_velocities);
			// 	group->sendCommand(groupCommand);
			// 	// Update the goal_normal
			// 	update_rel_goal();
			// 	// Use the transform to convert the received global impedance goal to the current, local goal in the world frame
			// 	goal_normal = odom_tf_current_rotation_matrix * global_goal_normal;
			// 	// Translate the global goal into homogeneous coordinates
			// 	Eigen::VectorXd impedance_global_goal_worker(4);
			// 	impedance_global_goal_worker << impedance_global_goal(0,2), 1;
			// 	// Transform the global impedance goal into the local frame
			// 	impedance_goal(0,2) = (odom_tf_current_homogeneous_matrix * impedance_global_goal_worker)(0,2);
			// 	// Set the current local goal
			// 	// Set the desired stabilization position as a point some distance into the target and 0 velocity
			// 	// Get the current end-effector position and add an offset to it along the normal
			// 	Eigen::Vector3d current_end_effector_position_worker;
			// 	Eigen::MatrixXd identity_worker(3,3);
			// 	identity_worker << MatrixXd::Identity(3,3);
			// 	identity_worker = impedance_goal_offset * identity_worker;
			// 	Eigen::Vector3d goal_offset_worker;
			// 	goal_offset_worker = identity_worker * goal_normal;
			// 	current_end_effector_position_worker = (robot_state->getGlobalLinkTransform(end_effector_name)).translation();
			// 	impedance_goal(0) = current_end_effector_position_worker.x() + goal_offset_worker(0);
			// 	impedance_goal(1) = current_end_effector_position_worker.y() + goal_offset_worker(1);
			// 	impedance_goal(2) = current_end_effector_position_worker.z() + goal_offset_worker(2);
			// 	// Convert the loccal goal to a global goal
			// 	// Get the transform from the local frame to the ODOM frame
			// 	listener_ptr -> lookupTransform(global_origin_frame_name, "world", ros::Time::now(), odom_tf_goal);
			// 	odom_tf_goal_translation << odom_tf_goal.getOrigin().getX(), odom_tf_goal.getOrigin().getY(), odom_tf_goal.getOrigin().getZ();
			// 	// Store the tf::Quaternion
			// 	tf::Quaternion tfQuat;
			// 	tfQuat = odom_tf_goal.getRotation();
			// 	// Convert the tf::Quaternion to an Eigen vector
			// 	Eigen::VectorXd odom_tf_goal_quaternion_worker(4);
			// 	odom_tf_goal_quaternion_worker << tfQuat.x(), tfQuat.y(), tfQuat.z(), tfQuat.w();
			// 	// Convert the Eigen vector to an Eigen quaternion
			// 	Eigen::Quaterniond odom_tf_goal_quaternion(odom_tf_goal_quaternion_worker(3),odom_tf_goal_quaternion_worker(0),odom_tf_goal_quaternion_worker(1),odom_tf_goal_quaternion_worker(2));
			// 	Eigen::Quaterniond odom_tf_goal_quaternion_normalized;
			// 	odom_tf_goal_quaternion_normalized = odom_tf_goal_quaternion.normalized();
			// 	odom_tf_goal_rotation_matrix = odom_tf_goal_quaternion_normalized.toRotationMatrix();	
			// 	odom_tf_goal_homogeneous_matrix << odom_tf_goal_rotation_matrix, odom_tf_goal_translation, 0, 0, 0, 1;
			// 	// Use the transform to convert the impedance goal to a global goal
			// 	// Translate the local goal into homogeneous coordinates
			// 	Eigen::VectorXd impedance_local_goal_worker(4);
			// 	impedance_local_goal_worker << impedance_goal(0,2), 1;
			// 	// Transform the local impedance goal into the global frame
			// 	impedance_global_goal(0,2) = (odom_tf_current_homogeneous_matrix * impedance_local_goal_worker)(0,2);
			// 	// Update the state to stabilization
			// 	state = 4;
			// 	ros::param::set("/tf_moveit_goalsetNode/manipulation_state", "impedance");
			// 	ROS_INFO("Running stabilization");
			// 	// Let the main_state_machine node know that the robot is ready for another command
			// 	status.data = 3;
			// 	state_input_pub.publish(status);
			// }
		}
		// Check if the robot is stabilizing (impedance control)
		if (state == 4)
		{
			// update_impedance_goal();
			// // Get the joint velocities
			// jointVelocityVect << hebiJointAngVelocities.at(0), hebiJointAngVelocities.at(1), hebiJointAngVelocities.at(2);
			// // compute fwd kinematics from hebi joints
			// robot_state->setJointGroupPositions(joint_model_group, hebiJointAngles);
			// end_effector_state = robot_state->getGlobalLinkTransform(end_effector_name);
			// // Update the Jacobian
			// robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, wristJacobian);
			// // Calculate the taskspace velocity
			// taskSpaceVelocityVect = wristJacobian * jointVelocityVect;
			// // Compute the errors
			// position_error << impedance_goal(0) - end_effector_state.translation().x(), impedance_goal(1) - end_effector_state.translation().y(), impedance_goal(2) - end_effector_state.translation().z();
			// velocity_error(0) = impedance_goal(3) - taskSpaceVelocityVect(0);
			// velocity_error(1) = impedance_goal(4) - taskSpaceVelocityVect(1);
			// velocity_error(2) = impedance_goal(5) - taskSpaceVelocityVect(2);
			// // Set the desired cartesian forces
			// desired_cartesian_forces(0) = impedance_position_gain(0) * position_error(0) + impedance_derivative_gain(0) * velocity_error(0);
			// desired_cartesian_forces(1) = impedance_position_gain(1) * position_error(1) + impedance_derivative_gain(1) * velocity_error(1);
			// desired_cartesian_forces(2) = impedance_position_gain(2) * position_error(2) + impedance_derivative_gain(2) * velocity_error(2);
			// // Convert desired cartesian forces to joint efforts
			// Eigen::MatrixXd jacobian_worker(6,group->size());
			// jacobian_worker = wristJacobian.transpose();

			// for (unsigned int it = 0; it < group->size(); it++)
			// {
			// 	desired_torques.effort.push_back(
			// jacobian_worker(0,it) * desired_cartesian_forces(0) + jacobian_worker(1,it) * desired_cartesian_forces(1) + jacobian_worker(2,it) * desired_cartesian_forces(2) + jacobian_worker(3,it) * desired_cartesian_forces(3) + jacobian_worker(4,it) * desired_cartesian_forces(4) + jacobian_worker(5,it) * desired_cartesian_forces(5));
			// }

			// // desired_torques.y = jacobian_worker(1,0) * desired_cartesian_forces(0) + jacobian_worker(1,1) * desired_cartesian_forces(0) + jacobian_worker(1,2) * desired_cartesian_forces(2) + jacobian_worker(1,3) * desired_cartesian_forces(3) + jacobian_worker(1,4) * desired_cartesian_forces(4) + jacobian_worker(1,5) * desired_cartesian_forces(5);
			// // desired_torques.z = jacobian_worker(2,0) * desired_cartesian_forces(0) + jacobian_worker(2,1) * desired_cartesian_forces(0) + jacobian_worker(2,2) * desired_cartesian_forces(2) + jacobian_worker(2,3) * desired_cartesian_forces(3) + jacobian_worker(2,4) * desired_cartesian_forces(4) + jacobian_worker(2,5) * desired_cartesian_forces(5);
			// // Send the desired joint torques to the motors
			// desired_efforts_pub.publish(desired_torques);
		}
		// Check if the robot is naively pulling from part (velocity control)
		if (state == 5)
		{
			// // Grab the current end-effector pose
			// naive_pull_current_pose = robot_state->getGlobalLinkTransform(end_effector_name);
			// // Check to see if the end-effector has moved far enough
			// if ((naive_pull_starting_pose.translation() - naive_pull_current_pose.translation()).norm() < naive_pull_offset)
			// {
			// 	// Update the Jacobian
			// 	robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, wristJacobian);
			// 	// Update the joint velocities
			// 	joint_velocities = wristJacobian.inverse() * desired_velocity;
			// 	groupCommand.setVelocity(joint_velocities);
			// 	group->sendCommand(groupCommand);
			// }
			// // If it has moved far enough away, return it to home
			// else
			// {
			// 	// Stop the end-effector
			// 	joint_velocities.setZero();
			// 	groupCommand.setVelocity(joint_velocities);
			// 	group->sendCommand(groupCommand);
			// 	// Update the state to returning to home
			// 	state = 6;
			// 	ros::param::set("/tf_moveit_goalsetNode/manipulation_state", "home");
			// 	// Let the main_state_machine node know that the robot is initializing
			// 	status.data = 1;
			// 	state_input_pub.publish(status);
			// 	ROS_INFO("Pull successful, returning to home");
			// }
		}
		// Check if the robot is returning to home
		if (state == 6)
		{
			// Let the main_state_machine node know that the robot is initializing
			status.data = 1;
			state_input_pub.publish(status);
			ROS_INFO("Planning to return home");
			// Set move group planning constraints
			move_group.setNumPlanningAttempts(num_attempts_return_home);
			move_group.setPlanningTime(moveit_planning_time_return_home);
			// Create and set planning goal
			robot_state::RobotState goal_state(robot_model);
			goal_state.setJointGroupPositions(joint_model_group, joint_group_positions);
			moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
			std::cout<<"joint_goal is: "<<std::endl;
			ROS_INFO_STREAM(joint_goal);
			plan_req.group_name = PLANNING_GROUP;
			plan_req.goal_constraints.clear();
			plan_req.goal_constraints.push_back(joint_goal);
			planning_pipeline_global_ptr->generatePlan(*psmPtr, plan_req, plan_res);
			// Store plan
			moveit_msgs::MotionPlanResponse response;
			response_ptr = &response;
			plan_res.getMessage(response);
			// moveit_plans_pub_ptr->publish(my_plan);
			// Set the trajectory and execute the plan
			my_plan.trajectory_ = response.trajectory;
			// Execute move
			move_group.execute(my_plan);
			prev_plan_res = response;
			// Let the main_state_machine node know that the robot is executing
			status.data = 2;
			state_input_pub.publish(status);
			ROS_INFO("Returning home");
			// Update the state to waiting
			state = 0;
			ros::param::set("/tf_moveit_goalsetNode/manipulation_state", "standby");
			// Let the main_state_machine node know that the robot is initializing
			status.data = 3;
			state_input_pub.publish(status);
			ROS_INFO("Return to home successful, waiting");
		}
	}
	// Shutdown alongside ROS
	return 0;
}
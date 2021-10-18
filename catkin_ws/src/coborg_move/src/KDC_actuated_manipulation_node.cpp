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
#include "goal_getter/goal_msg.h"
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

// HEBI Includes
#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include "hebi_cpp_api/trajectory.hpp"

// TO DO:
// Get the frame and time stamp from the updated goal_getter message
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

// Initialize variables
// HEBI Initializations
Eigen::Vector3d motor_joints;
geometry_msgs::Twist publishState;
geometry_msgs::Vector3 torqueVect;
std::shared_ptr<hebi::Group> group;
// MoveIt Global Pointers
planning_scene_monitor::PlanningSceneMonitorPtr* psmPtr;
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
int state = 0;
std_msgs::Int32 status;
const std::string PLANNING_GROUP = "coborg_arm";
// Initialize Goal and Transform Variables
tf::TransformListener listener;
tf::StampedTransform odom_tf_goal;
std::string goal_frame = "/world";
ros::Time goal_time;
Eigen::Vector4d homogeneous_goal;
Eigen::Vector3d goal_normal;
Eigen::Vector3d goal_normal_x_axis;
Eigen::Vector3d goal_normal_y_axis;
Eigen::Vector3d goal_normal_z_axis;
Eigen::Matrix3d goal_normal_rotation_matrix;
Eigen::Vector3d odom_tf_goal_translation;
Eigen::Matrix3d odom_tf_goal_rotation_matrix;
Eigen::Matrix4d odom_tf_goal_homogeneous_matrix;
Eigen::Vector3d global_goal;
Eigen::Vector3d global_goal_normal;
std::vector<double> goal_tolerance_pose_default {0.02, 0.02, 0.02};
std::vector<double> goal_tolerance_pose = goal_tolerance_pose_default;
std::vector<double> goal_tolerance_angle_default {10, 0.79, 0.79};
std::vector<double> goal_tolerance_angle = goal_tolerance_angle_default;
double goal_tolerance_pose_adjustment = 0.025;
double goal_tolerance_pose_adjusted_threshold = 0.1;
float goal_offset = 0.15;
tf::StampedTransform odom_tf_current;
Eigen::Vector3d odom_tf_current_translation;
Eigen::Matrix3d odom_tf_current_rotation_matrix;
Eigen::Matrix4d odom_tf_current_homogeneous_matrix;
// Initialize Planning Variables
planning_interface::MotionPlanRequest plan_req;
planning_interface::MotionPlanResponse plan_res;
moveit_msgs::MotionPlanResponse prev_plan_res;
geometry_msgs::PoseStamped goal_pose;
moveit::planning_interface::MoveItErrorCode moveitSuccess;
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
ros::Time plan_start;
ros::Time plan_end;
double planning_time_offset_default = 0.3;
double planning_time_offset = planning_time_offset_default;
double stitching_time_offset_default = 0.1;
double stitching_time_offset = stitching_time_offset_default;
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
Eigen::VectorXd joint_velocities(group->size());
Eigen::VectorXd end_effector_force(6);
hebi::GroupCommand groupCommand(group->size());
hebi::GroupFeedback group_feedback(group->size());
Eigen::VectorXd motor_torques(group->size());
// Initialize Stabilization Variables
Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
std::vector<double> hebiJointAngles(3);
std::vector<double> hebiJointAngVelocities(3);
Eigen::Vector3d current_hebi_joints;
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
geometry_msgs::Vector3 desired_torques;
float baseMaxTorque = 2.0;
float elbowMaxTorque = 2.0;
float wristMaxTorque = 2.0;
// Initialize Naive Pull Variables
double naive_pull_offset = 0.1;
Eigen::Isometry3d naive_pull_starting_pose;
Eigen::Isometry3d naive_pull_current_pose;
// Initialize Return to Home Variables

// Define Functions
// Function to update global goal to local and fill in goal_pose message
void update_rel_goal()
{
	// Get the current transform from the ODOM frame to the world frame
	listener.lookupTransform("/world", "/odom", ros::Time::now(), odom_tf_current);
	odom_tf_current_translation << odom_tf_current.getOrigin().getX(), odom_tf_current.getOrigin().getY(), odom_tf_current.getOrigin().getZ();
	// Store the tf::Quaternion
	tf::Quaternion tfQuat;
	tfQuat = odom_tf_current.getRotation();
	// Convert the tf::Quaternion to an Eigen vector
	Eigen::VectorXd odom_tf_current_quaternion_worker(4);
	odom_tf_current_quaternion_worker << tfQuat.x(), tfQuat.y(), tfQuat.z(), tfQuat.w();
	// Convert the Eigen vector to an Eigen quaternion
	Eigen::Quaterniond odom_tf_current_quaternion(odom_tf_current_quaternion_worker(3),odom_tf_current_quaternion_worker(0),odom_tf_current_quaternion_worker(1),odom_tf_current_quaternion_worker(2));
	Eigen::Quaterniond odom_tf_current_quaternion_normalized;
	odom_tf_current_quaternion_normalized = odom_tf_current_quaternion.normalized();
	odom_tf_current_rotation_matrix = odom_tf_current_quaternion_normalized.toRotationMatrix();
	odom_tf_current_homogeneous_matrix << odom_tf_current_rotation_matrix, odom_tf_current_translation, (double)0, (double)0, (double)0, (double)1;
	// Use the transform to convert the received global goal to the current, local goal in the world frame
	goal_normal = odom_tf_current_rotation_matrix * global_goal_normal;
	Eigen::Vector4d global_goal_homogeneous_worker;
	global_goal_homogeneous_worker << global_goal, 1;
	homogeneous_goal = odom_tf_current_homogeneous_matrix * global_goal_homogeneous_worker;
	// Fill geometry pose message with goal location information
	goal_pose.header.frame_id = goal_frame;
	goal_pose.pose.position.x = homogeneous_goal(0);
	goal_pose.pose.position.y = homogeneous_goal(1);
	goal_pose.pose.position.z = homogeneous_goal(2);
	// Create goal rotation matrix from x-axis (goal_normal) and two random, perpendicular axes
	// Set x axis to be aligned with the norm so that the end-effector pushes against the plate
	goal_normal_x_axis = goal_normal.normalized();
	// Set the z axis temporarily to a random variable
	goal_normal_z_axis << 1, 1, 1;
	// Set the y axis to be perpendicular to the x axis and the temporary z axis
	goal_normal_y_axis = goal_normal_x_axis.cross(goal_normal_z_axis.normalized());
	// If the x and temporary z axes happened to be the same, the y axis will be 0, so retry with a new temporary vector
	Eigen::Vector3d zero;
	if (goal_normal_y_axis == zero.setZero())
	{
		// Set the z axis to a different, temporary random variable
		goal_normal_z_axis << 0, 1, 1;
		// Set the y axis to be perpendicular to the x axis and the new temporary z axis
		goal_normal_y_axis = goal_normal_x_axis.cross(goal_normal_z_axis.normalized());
	}
	// Set the z axis to be perpendicular to the xaxis and the y axis
	goal_normal_z_axis = goal_normal_x_axis.cross(goal_normal_y_axis.normalized());
	// Fill in the rotation matrix with the normalized x, y, and z axes
	goal_normal_rotation_matrix << goal_normal_x_axis.normalized(), goal_normal_y_axis.normalized(), goal_normal_z_axis.normalized();
	// Convert the rotation matrix to a quaternion
	Eigen::Matrix3f goal_normal_rotation_matrix_worker;
	Eigen::Quaternionf goal_normal_quaternion(goal_normal_rotation_matrix_worker);
	// Fill geometry pose message with goal orientation information
	goal_pose.pose.orientation.x = goal_normal_quaternion.x();
	goal_pose.pose.orientation.y = goal_normal_quaternion.y();
	goal_pose.pose.orientation.z = goal_normal_quaternion.z();
	goal_pose.pose.orientation.w = goal_normal_quaternion.w();
	return;
}
void update_impedance_goal()
{
	// Get the current transform from the ODOM frame to the world frame
	listener.lookupTransform("/world", "/odom", ros::Time::now(), odom_tf_current_impedance);
	odom_tf_current_translation_impedance << odom_tf_current_impedance.getOrigin().getX(), odom_tf_current_impedance.getOrigin().getY(), odom_tf_current_impedance.getOrigin().getZ();
	// Store the tf::Quaternion
	tf::Quaternion tfQuat_impedance;
	tfQuat_impedance = odom_tf_current_impedance.getRotation();
	// Convert the tf::Quaternion to an Eigen vector
	Eigen::VectorXd odom_tf_current_quaternion_impedance_worker(4);
	odom_tf_current_quaternion_impedance_worker << tfQuat_impedance.x(), tfQuat_impedance.y(), tfQuat_impedance.z(), tfQuat_impedance.w();
	// Convert the Eigen vector to an Eigen quaternion
	Eigen::Quaterniond odom_tf_current_quaternion_impedance(odom_tf_current_quaternion_impedance_worker(3),odom_tf_current_quaternion_impedance_worker(0),odom_tf_current_quaternion_impedance_worker(1),odom_tf_current_quaternion_impedance_worker(2));
	Eigen::Quaterniond odom_tf_current_quaternion_impedance_normalized;
	odom_tf_current_quaternion_impedance_normalized = odom_tf_current_quaternion_impedance.normalized();
	odom_tf_current_rotation_matrix_impedance = odom_tf_current_quaternion_impedance_normalized.toRotationMatrix();	
	odom_tf_current_homogeneous_matrix_impedance << odom_tf_current_rotation_matrix_impedance, odom_tf_current_translation_impedance, 0, 0, 0, 1;
	// Use the transform to convert the received global impedance goal to the current, local goal in the world frame
	goal_normal = odom_tf_current_rotation_matrix_impedance * global_goal_normal;
	// Translate the global goal into homogeneous coordinates
	Eigen::VectorXd impedance_global_goal_homogeneous_worker(4);
	impedance_global_goal_homogeneous_worker << impedance_global_goal(0,2), 1;
	// Transform the global impedance goal into the local frame
	impedance_goal(0,2) = (odom_tf_current_homogeneous_matrix * impedance_global_goal_homogeneous_worker)(0,2);
	return;
}

// Define subscriber callbacks
// Camera goal callback
void camera_goal_callback(const goal_getter::goal_msg::ConstPtr& msg)
{
	if (state == 1)
	{
		// Block other camera inputs
		state = -1;
		// Reset the planning and stitching time offsets from the previous action
		planning_time_offset = planning_time_offset_default;
		stitching_time_offset = stitching_time_offset_default;
		// Set the time from the message header
		//goal_time = msg->header.stamp;//////////////////////FIX///////////////////////
		goal_time = ros::Time::now();
		// Convert the received goal to eigen
		goal_normal << msg->normal_x, msg->normal_y, msg->normal_z;
		homogeneous_goal << msg->x, msg->y, msg->z;
		// Get the transform from the local frame to the ODOM frame
		//odom_tf_goal = listener.waitForTransform("/odom", msg->header.frame_id, goal_time, ros::Duration(3.0));/////////FIX/////////////////
		listener.lookupTransform("/odom", "/d400_link", goal_time, odom_tf_goal);
		odom_tf_goal_translation << odom_tf_goal.getOrigin().getX(), odom_tf_goal.getOrigin().getY(), odom_tf_goal.getOrigin().getZ();
		// Store the tf::Quaternion
		tf::Quaternion tfQuat_initial;
		tfQuat_initial = odom_tf_goal.getRotation();
		// Convert the tf::Quaternion to an Eigen vector
		Eigen::VectorXd odom_tf_goal_quaternion_worker(4);
		odom_tf_goal_quaternion_worker << tfQuat_initial.x(), tfQuat_initial.y(), tfQuat_initial.z(), tfQuat_initial.w();
		// Convert the Eigen vector to an Eigen quaternion
		Eigen::Quaterniond odom_tf_goal_quaternion(odom_tf_goal_quaternion_worker(3),odom_tf_goal_quaternion_worker(0),odom_tf_goal_quaternion_worker(1),odom_tf_goal_quaternion_worker(2));
		Eigen::Quaterniond odom_tf_goal_quaternion_normalized;
		odom_tf_goal_quaternion_normalized = odom_tf_goal_quaternion.normalized();
		odom_tf_goal_rotation_matrix = odom_tf_goal_quaternion_normalized.toRotationMatrix();
		odom_tf_goal_homogeneous_matrix << odom_tf_goal_rotation_matrix, odom_tf_goal_translation, 0, 0, 0, 1;
		// Use the transform to convert the received goal to a global goal
		global_goal_normal = odom_tf_goal_rotation_matrix * goal_normal;
		Eigen::VectorXd goal_normal_homogeneous_worker(4);
		goal_normal_homogeneous_worker << goal_normal(0)*goal_offset, goal_normal(1)*goal_offset, goal_normal(2)*goal_offset, 1;
		Eigen::VectorXd homogeneous_global_goal(4);
		homogeneous_global_goal = odom_tf_goal_homogeneous_matrix * (homogeneous_goal - goal_normal_homogeneous_worker);
		global_goal << homogeneous_global_goal(0), homogeneous_global_goal(1), homogeneous_global_goal(2);
		// Update the relative goal based on the robot's global position
		update_rel_goal();
		// Plan RRT Connect Path and send it for execution
		// Reset tolerances
		goal_tolerance_pose = goal_tolerance_pose_default;
		goal_tolerance_angle = goal_tolerance_angle_default;
		// Set kinematic constraints
		moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("end_link/INPUT_INTERFACE", goal_pose, goal_tolerance_pose, goal_tolerance_angle);
		plan_req.group_name = PLANNING_GROUP;
		plan_req.goal_constraints.clear();
		plan_req.goal_constraints.push_back(pose_goal);
		// Lock the visual planner
		planning_scene_monitor::LockedPlanningSceneRO lscene(*psmPtr);
		/* Now, call the pipeline and check whether planning was successful. */
		planning_pipeline_global_ptr->generatePlan(lscene, plan_req, plan_res);
		/* Now, call the pipeline and check whether planning was successful. */
		/* Check that the planning was successful */
		while (plan_res.error_code_.val != plan_res.error_code_.SUCCESS)
		{
			ROS_ERROR("Could not compute plan successfully, increasing tolerances");
			// Increment goal tolerance and break out if tolerances are too large
			goal_tolerance_pose[0] = goal_tolerance_pose[0] + goal_tolerance_pose_adjustment;
			goal_tolerance_pose[1] = goal_tolerance_pose[1] + goal_tolerance_pose_adjustment;
			goal_tolerance_pose[2] = goal_tolerance_pose[2] + goal_tolerance_pose_adjustment;
			moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("end_link/INPUT_INTERFACE", goal_pose, goal_tolerance_pose, goal_tolerance_angle);
			plan_req.group_name = PLANNING_GROUP;
			plan_req.goal_constraints.clear();
			plan_req.goal_constraints.push_back(pose_goal);
			planning_scene_monitor::LockedPlanningSceneRO lscene(*psmPtr);
			planning_pipeline_global_ptr->generatePlan(lscene, plan_req, plan_res);
			if ( sqrt(pow(goal_tolerance_pose[0],2)+pow(goal_tolerance_pose[1],2)+pow(goal_tolerance_pose[2],2)) > goal_tolerance_pose_adjusted_threshold)
			{
				state = 0;
				ROS_INFO("Couldn't find a suitable path, returning to waiting");
				return;
			}
		}
		/////////////// Visualize the result
		moveit_msgs::DisplayTrajectory display_trajectory;
		/* Visualize the trajectory */
		ROS_INFO("Visualizing the trajectory");
		moveit_msgs::MotionPlanResponse response;
		response_ptr = &response;
		plan_res.getMessage(response);

		display_trajectory.trajectory_start = response.trajectory_start;
		display_trajectory.trajectory.clear();
		display_trajectory.trajectory.push_back(response.trajectory);
		display_publisher_ptr->publish(display_trajectory);
		visual_tools_ptr->publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
		visual_tools_ptr->trigger();
		///////////////
		// Set the trajectory and execute the plan
		my_plan.trajectory_ = response.trajectory;
		moveitSuccess = move_group_ptr->plan(my_plan);
		move_group_ptr->asyncExecute(my_plan);
		ros::param::set("/tf_moveit_goalsetNode/manipulation_state", "standby");
		prev_plan_res = response;
		plan_start = ros::Time::now();
		state = 2;
		status.data = 2;
		state_input_pub_ptr->publish(status);
		ROS_INFO("Moving to target");
	}
}
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
			// Update the normal vector
			update_rel_goal();
			// Set the desired end-effector velocity
			desired_velocity << -goal_normal * naive_push_speed, 0, 0, 0;
			// Grab the starting end-effector pose
			naive_pull_starting_pose = (*robot_state_ptr)->getGlobalLinkTransform("end_link/INPUT_INTERFACE");
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
	// Check if the trajectory has finished
	if (msg->feedback.state == "IDLE")
	{
		// Check if the robot was moving to a target
		if (state == 2)
		{
			state = 3;
			ROS_INFO("Successfully reached target offset; extending");
			ros::param::set("/tf_moveit_goalsetNode/manipulation_state", "velocity");
			// Update the normal vector
			update_rel_goal();
			// Set the desired end-effector velocity
			desired_velocity << goal_normal * naive_push_speed, 0, 0, 0;
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
// HEBI joint angles callback
void hebiJointsCallback(const geometry_msgs::Twist::ConstPtr & hebimsg)
{
	// Grab the joint angles and velocities of the robot
	hebiJointAngles.at(0) = hebimsg->linear.x;
	hebiJointAngles.at(1) = hebimsg->linear.y;
	hebiJointAngles.at(2) = hebimsg->linear.z;
	hebiJointAngVelocities.at(0) = hebimsg->angular.x;
	hebiJointAngVelocities.at(1) = hebimsg->angular.y;
	hebiJointAngVelocities.at(2) = hebimsg->angular.z;
}

int main(int argc, char** argv)
{
	// Set initial matrix values
	joint_velocities.setZero();
	desired_velocity.setZero();
	impedance_goal.setZero();
	impedance_global_goal.setZero();
	desired_cartesian_forces.setZero();
	impedance_position_gain << 50, 0, 0; // X is the normal direction
	impedance_derivative_gain << 2, 2, 2; // X is the normal direction
	
	// Initialize ROS
	ros::init(argc, argv, "KDC_actuated_manipulation_node");
	ros::Duration(10).sleep();

	// Construct node
	ros::NodeHandle node_handle;
	ros::NodeHandle* node_handle_ptr;
	node_handle_ptr = &node_handle;

	ros::AsyncSpinner spinner(0);
	spinner.start();

	// MoveIt initializations and declarations
	//////////////////////////////////////////////////////////////
	// MoveIt Motion Planning Pipeline Tutorial Initializations
	// Load Robot Model
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	move_group_ptr = &move_group;
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model_loader::RobotModelLoaderPtr robot_model_loader_ptr(new robot_model_loader::RobotModelLoader("robot_description"));
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
	// Create Planning Scene
	planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader_ptr));
	psmPtr = &psm;
	/* listen for planning scene messages on topic /XXX and apply them to the internal planning scene
						the internal planning scene accordingly */
	psm->startSceneMonitor();
	/* listens to changes of world geometry, collision objects, and (optionally) octomaps
								world geometry, collision objects and optionally octomaps */
	psm->startWorldGeometryMonitor();
	/* listen to joint state updates as well as changes in attached collision objects
						and update the internal planning scene accordingly*/
	psm->startStateMonitor();

	/* We can also use the RobotModelLoader to get a robot model which contains the robot's kinematic information */
	moveit::core::RobotModelPtr kin_model = robot_model_loader_ptr->getModel();
	////////////////////////////////////////////////////////CHANGED THE ABOVE FROM ROBOT_MODEL TO KIN_MODEL
	/* We can get the most up to date robot state from the PlanningSceneMonitor by locking the internal planning scene
	for reading. This lock ensures that the underlying scene isn't updated while we are reading it's state.
	RobotState's are useful for computing the forward and inverse kinematics of the robot among many other uses */
	moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));
	robot_state_ptr = &robot_state;
	/* Create a JointModelGroup to keep track of the current robot pose and planning group. The Joint Model
	group is useful for dealing with one set of joints at a time such as a left arm or a end effector */
	joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
	// Create Planning Pipeline
	planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, *node_handle_ptr, "planning_plugin", "request_adapters"));
	planning_pipeline_global_ptr = planning_pipeline;
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
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
	//////////////////////////////////////////////////////////////

	// Initialize Publishers
	ros::Publisher state_input_pub = node_handle.advertise<std_msgs::Int32>("/state_input", 5);
	state_input_pub_ptr = &state_input_pub;
	ros::Publisher desired_efforts_pub = node_handle.advertise<geometry_msgs::Vector3>("desired_hebi_efforts", 1);
	ros::Publisher display_publisher = node_handle_ptr->advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	display_publisher_ptr = &display_publisher;

	// Publish initial state (waiting)
	status.data = 3;
	state_input_pub.publish(status);

	// Initialize Subscribers
	ros::Subscriber camera_goal_sub = node_handle.subscribe("/goal", 1, camera_goal_callback);
	ros::Subscriber state_output_sub = node_handle.subscribe("/state_output", 1, state_output_callback);
	ros::Subscriber execute_action_sub = node_handle.subscribe("/execute_trajectory/feedback", 5, execute_trajectory_feedback_callback);
	ros::Subscriber hebi_joints_sub = node_handle.subscribe("hebi_joints", 1, hebiJointsCallback);

	while (ros::ok())
	{
		// Check if the robot is executing trajectories to target
		if (state == 2)
		{
			// prev_plan_res contains the current, time-stamped RRT plan
			// Determine how far into the future of the current plan to begin the next plan
			desired_plan_start_time = planning_time_offset + stitching_time_offset + ros::Time::now().toSec() - plan_start.toSec();
			// Loop through the previously planned path until you find the point in the path corresponding to the desired time
			trajectory_start_point_success = 0;
			for (unsigned int i = 0; i < (sizeof(prev_plan_res.trajectory.joint_trajectory.points) / sizeof(prev_plan_res.trajectory.joint_trajectory.points[0])); ++i)
			{
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
					// Stop looping
					break;
				}
			}
			// If desired_plan_start_time is greater than the time to finish the path, don't bother planning a new one
			if (!trajectory_start_point_success)
			{
				continue;
			}
			// Use RRT to plan a path from the beginning point just extracted to the current goal point, updated based on the t_265 localization
			// Update the relative goal based on the robot's global position
			update_rel_goal();
			// Plan RRT Connect Path and send it for execution
			// Set kinematic constraints
			moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("end_link/INPUT_INTERFACE", goal_pose, goal_tolerance_pose, goal_tolerance_angle);
			plan_req.group_name = PLANNING_GROUP;
			plan_req.goal_constraints.clear();
			plan_req.goal_constraints.push_back(pose_goal);
			// Lock the visual planner
			{
				planning_scene_monitor::LockedPlanningSceneRO lscene(*psmPtr);
				/* Now, call the pipeline and check whether planning was successful. */
				planning_pipeline->generatePlan(lscene, plan_req, plan_res);
			}
			/* Now, call the pipeline and check whether planning was successful. */
			/* Check that the planning was successful */
			if (plan_res.error_code_.val != plan_res.error_code_.SUCCESS)
			{
				ROS_ERROR("Could not compute plan successfully");
				continue;
				// CONSIDER ADDING IN THE TOLERANCES INCREASES
			}
			// Stitch the new response (response.trajectory.joint_trajectory.points) onto the beginning of the old one
			stitch_plan_start = ros::Time::now();
			// Record the memory size of the arrays
			int new_trajectory_length = sizeof(response_ptr->trajectory.joint_trajectory.points) / sizeof(response_ptr->trajectory.joint_trajectory.points[0]);
			// Create a working array to hold the stitched trajectory
			std::vector<trajectory_msgs::JointTrajectoryPoint> working_trajectory_array;
			//std::cout << prev_plan_res.trajectory.joint_trajectory.points.begin();
			std::copy(prev_plan_res.trajectory.joint_trajectory.points.begin(), prev_plan_res.trajectory.joint_trajectory.points.begin() + trajectory_start_point, working_trajectory_array.begin());
			std::copy(response_ptr->trajectory.joint_trajectory.points.begin(), response_ptr->trajectory.joint_trajectory.points.begin() + new_trajectory_length, working_trajectory_array.begin() + trajectory_start_point);
			// Loop through the full working trajectory and find the new starting point (updated for how far the robot has traversed)
			// Update the time stamps accordingly
			for (unsigned int j = 0; j < (trajectory_start_point + new_trajectory_length); ++j)
			{
				// Check if the planning has taken so long that the manipulator has already reached the new trajectory, stop trying to fix the trajectory and restart
				if ((j > trajectory_start_point) && (new_plan_origin_found == 0))
				{
					// If there wasn't enough time to plan the trajectory, increase the planning time to the total time that it took to plan + 5%
					planning_time_offset = (ros::Time::now() - plan_start).toSec() * 1.05;
					ROS_INFO("Failed to plan in time, new planning_time_offset:%s", planning_time_offset);
					break;
				}
				// If a starting point in the old trajectory has been found, set it as the new starting point for the final, stitched and cut trajectory
				if ((new_plan_origin_found == 0) && (working_trajectory_array[j].time_from_start.toSec() > ((ros::Time::now() - plan_start).toSec() + stitching_time_offset)))
				{
					// Record the cut point
					new_plan_origin = j;
					new_plan_origin_found = 1;
					// Record the time difference
					stitching_plan_time_diff = -working_trajectory_array[j].time_from_start;
					working_trajectory_array[j].time_from_start = ros::Duration(0.0);
				}
				// If a starting point was found in a previous loop, update the time from start of this point
				else if (new_plan_origin_found == 1)
				{
					// Subtract the time already elapsed to get the new time from start
					working_trajectory_array[j].time_from_start = working_trajectory_array[j].time_from_start + ros::Duration(stitching_plan_time_diff);
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
				// Update the trajectory
				std::copy(working_trajectory_array.begin()+new_plan_origin, working_trajectory_array.end(), response_ptr->trajectory.joint_trajectory.points.begin());
				/////////////// Visualize the result
				//ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
				moveit_msgs::DisplayTrajectory display_trajectory;

				/* Visualize the trajectory */
				ROS_INFO("Visualizing the trajectory");
				moveit_msgs::MotionPlanResponse response;
				plan_res.getMessage(response);

				display_trajectory.trajectory_start = response.trajectory_start;
				display_trajectory.trajectory.clear();
				display_trajectory.trajectory.push_back(response.trajectory);
				display_publisher.publish(display_trajectory);
				visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
				visual_tools.trigger();
				///////////////
				// Update and Execute the plan
				my_plan.trajectory_ = response.trajectory;
				moveitSuccess = move_group.plan(my_plan);
				move_group.asyncExecute(my_plan);
				prev_plan_res = response;
				ROS_INFO("Plan successfully executed.  Time to plan: %s", (ros::Time::now() - plan_start).toSec());
				plan_start = ros::Time::now();
			}
			else
			{
				// If there wasn't enough time to stitch the trajectories together, increase the stitching time to the total time that it took to stitch + 5%
				stitching_time_offset = ((ros::Time::now() - plan_start).toSec() - planning_time_offset) * 1.05;
				ROS_INFO("Failed to stitch in time, new stitching_time_offset:%s", stitching_time_offset);
			}
			new_plan_origin_found = 0;
		}
		// Check if the robot is naively moving toward the target (admittance control)
		if (state == 3)
		{
			// Update the Jacobian
			robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, wristJacobian);
			// Get the joint forces
			if (group->getNextFeedback(group_feedback))
			{
				motor_torques = group_feedback.getEffort();
			}
			// Update the end-effector force
			end_effector_force = wristJacobian * motor_torques;
			// Check to see if the force on the end-effector is below the threshold
			if (sqrt(pow(end_effector_force(0),2)+pow(end_effector_force(1),2)+pow(end_effector_force(2),2)) < naive_push_force_threshold)
			{
				// Update the joint velocities
				joint_velocities = wristJacobian.inverse() * desired_velocity;
				groupCommand.setVelocity(joint_velocities);
				group->sendCommand(groupCommand);
			}
			// If it exceeds the threshold, set up impedance control
			else
			{
				// Stop the end-effector
				joint_velocities.setZero();
				groupCommand.setVelocity(joint_velocities);
				group->sendCommand(groupCommand);
				// Update the goal_normal
				update_rel_goal();
				// Use the transform to convert the received global impedance goal to the current, local goal in the world frame
				goal_normal = odom_tf_current_rotation_matrix * global_goal_normal;
				// Translate the global goal into homogeneous coordinates
				Eigen::VectorXd impedance_global_goal_worker(4);
				impedance_global_goal_worker << impedance_global_goal(0,2), 1;
				// Transform the global impedance goal into the local frame
				impedance_goal(0,2) = (odom_tf_current_homogeneous_matrix * impedance_global_goal_worker)(0,2);
				// Set the current local goal
				// Set the desired stabilization position as a point some distance into the target and 0 velocity
				// Get the current end-effector position and add an offset to it along the normal
				Eigen::Vector3d current_end_effector_position_worker;
				Eigen::MatrixXd identity_worker(3,3);
				identity_worker << MatrixXd::Identity(3,3);
				identity_worker = impedance_goal_offset * identity_worker;
				Eigen::Vector3d goal_offset_worker;
				goal_offset_worker = identity_worker * goal_normal;
				current_end_effector_position_worker = (robot_state->getGlobalLinkTransform("end_link/INPUT_INTERFACE")).translation();
				impedance_goal(0) = current_end_effector_position_worker.x() + goal_offset_worker(0);
				impedance_goal(1) = current_end_effector_position_worker.y() + goal_offset_worker(1);
				impedance_goal(2) = current_end_effector_position_worker.z() + goal_offset_worker(2);
				// Convert the loccal goal to a global goal
				// Get the transform from the local frame to the ODOM frame
				listener.lookupTransform("/odom", "world", ros::Time::now(), odom_tf_goal);
				odom_tf_goal_translation << odom_tf_goal.getOrigin().getX(), odom_tf_goal.getOrigin().getY(), odom_tf_goal.getOrigin().getZ();
				// Store the tf::Quaternion
				tf::Quaternion tfQuat;
				tfQuat = odom_tf_goal.getRotation();
				// Convert the tf::Quaternion to an Eigen vector
				Eigen::VectorXd odom_tf_goal_quaternion_worker(4);
				odom_tf_goal_quaternion_worker << tfQuat.x(), tfQuat.y(), tfQuat.z(), tfQuat.w();
				// Convert the Eigen vector to an Eigen quaternion
				Eigen::Quaterniond odom_tf_goal_quaternion(odom_tf_goal_quaternion_worker(3),odom_tf_goal_quaternion_worker(0),odom_tf_goal_quaternion_worker(1),odom_tf_goal_quaternion_worker(2));
				Eigen::Quaterniond odom_tf_goal_quaternion_normalized;
				odom_tf_goal_quaternion_normalized = odom_tf_goal_quaternion.normalized();
				odom_tf_goal_rotation_matrix = odom_tf_goal_quaternion_normalized.toRotationMatrix();	
				odom_tf_goal_homogeneous_matrix << odom_tf_goal_rotation_matrix, odom_tf_goal_translation, 0, 0, 0, 1;
				// Use the transform to convert the impedance goal to a global goal
				// Translate the local goal into homogeneous coordinates
				Eigen::VectorXd impedance_local_goal_worker(4);
				impedance_local_goal_worker << impedance_goal(0,2), 1;
				// Transform the local impedance goal into the global frame
				impedance_global_goal(0,2) = (odom_tf_current_homogeneous_matrix * impedance_local_goal_worker)(0,2);
				// Update the state to stabilization
				state = 4;
				ros::param::set("/tf_moveit_goalsetNode/manipulation_state", "impedance");
				ROS_INFO("Running stabilization");
				// Let the main_state_machine node know that the robot is ready for another command
				status.data = 3;
				state_input_pub.publish(status);
			}
		}
		// Check if the robot is stabilizing (impedance control)
		if (state == 4)
		{
			update_impedance_goal();
			// Get the joint velocities
			jointVelocityVect << hebiJointAngVelocities.at(0), hebiJointAngVelocities.at(1), hebiJointAngVelocities.at(2);
			// compute fwd kinematics from hebi joints
			robot_state->setJointGroupPositions(joint_model_group, hebiJointAngles);
			end_effector_state = robot_state->getGlobalLinkTransform("end_link/INPUT_INTERFACE");
			// Update the Jacobian
			robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, wristJacobian);
			// Calculate the taskspace velocity
			taskSpaceVelocityVect = wristJacobian * jointVelocityVect;
			// Compute the errors
			position_error << impedance_goal(0) - end_effector_state.translation().x(), impedance_goal(1) - end_effector_state.translation().y(), impedance_goal(2) - end_effector_state.translation().z();
			velocity_error(0) = impedance_goal(3) - taskSpaceVelocityVect(0);
			velocity_error(1) = impedance_goal(4) - taskSpaceVelocityVect(1);
			velocity_error(2) = impedance_goal(5) - taskSpaceVelocityVect(2);
			// Set the desired cartesian forces
			desired_cartesian_forces(0) = impedance_position_gain(0) * position_error(0) + impedance_derivative_gain(0) * velocity_error(0);
			desired_cartesian_forces(1) = impedance_position_gain(1) * position_error(1) + impedance_derivative_gain(1) * velocity_error(1);
			desired_cartesian_forces(2) = impedance_position_gain(2) * position_error(2) + impedance_derivative_gain(2) * velocity_error(2);
			// Convert desired cartesian forces to joint efforts
			Eigen::MatrixXd jacobian_worker(3,6);
			jacobian_worker = wristJacobian.transpose();
			desired_torques.x = jacobian_worker(0,0) * desired_cartesian_forces(0) + jacobian_worker(0,1) * desired_cartesian_forces(0) + jacobian_worker(0,2) * desired_cartesian_forces(2) + jacobian_worker(0,3) * desired_cartesian_forces(3) + jacobian_worker(0,4) * desired_cartesian_forces(4) + jacobian_worker(0,5) * desired_cartesian_forces(5);
			desired_torques.y = jacobian_worker(1,0) * desired_cartesian_forces(0) + jacobian_worker(1,1) * desired_cartesian_forces(0) + jacobian_worker(1,2) * desired_cartesian_forces(2) + jacobian_worker(1,3) * desired_cartesian_forces(3) + jacobian_worker(1,4) * desired_cartesian_forces(4) + jacobian_worker(1,5) * desired_cartesian_forces(5);
			desired_torques.z = jacobian_worker(2,0) * desired_cartesian_forces(0) + jacobian_worker(2,1) * desired_cartesian_forces(0) + jacobian_worker(2,2) * desired_cartesian_forces(2) + jacobian_worker(2,3) * desired_cartesian_forces(3) + jacobian_worker(2,4) * desired_cartesian_forces(4) + jacobian_worker(2,5) * desired_cartesian_forces(5);
			// Send the desired joint torques to the motors
			desired_efforts_pub.publish(desired_torques);
		}
		// Check if the robot is naively pulling from part (velocity control)
		if (state == 5)
		{
			// Grab the current end-effector pose
			naive_pull_current_pose = robot_state->getGlobalLinkTransform("end_link/INPUT_INTERFACE");;
			// Check to see if the end-effector has moved far enough
			if ((naive_pull_starting_pose.translation() - naive_pull_current_pose.translation()).norm() < naive_pull_offset)
			{
				// Update the Jacobian
				robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, wristJacobian);
				// Update the joint velocities
				joint_velocities = wristJacobian.inverse() * desired_velocity;
				groupCommand.setVelocity(joint_velocities);
				group->sendCommand(groupCommand);
			}
			// If it has moved far enough away, return it to home
			else
			{
				// Stop the end-effector
				joint_velocities.setZero();
				groupCommand.setVelocity(joint_velocities);
				group->sendCommand(groupCommand);
				// Update the state to returning to home
				state = 6;
				ros::param::set("/tf_moveit_goalsetNode/manipulation_state", "home");
				// Let the main_state_machine node know that the robot is initializing
				status.data = 1;
				state_input_pub.publish(status);
				ROS_INFO("Pull successful, returning to home");
			}
		}
		// Check if the robot is returning to home
		if (state == 6)
		{
			// Let the main_state_machine node know that the robot is initializing
			status.data = 1;
			state_input_pub.publish(status);
			ROS_INFO("Planning to return home");
			// Set home joint positions
			std::vector<double> joint_group_positions = { -1.91986, -2.37365, -2.51327 };
			// Set a joint target in MoveIt
			move_group.setJointValueTarget(joint_group_positions);
			// Check whether planning was successful
			moveitSuccess = move_group.plan(my_plan);
			// Execute move
			move_group.execute(my_plan);
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
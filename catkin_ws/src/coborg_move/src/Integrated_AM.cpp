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
	// In a separate terminal, run "rostopic echo /main_cmd"
	// In a separate terminal, run "rostopic echo /execute_trajectory/feedback"
	// In a separate terminal, run "rostopic echo /hebi_joints"


// Feng Xiang and Yuqing Qin and Gerry D'Ascoli and Jonathan Lord Fonda:


// C++ and ROS Includes
#include <string>
#include <math.h>
#include <algorithm>
#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/JointState.h>
#include "std_srvs/Empty.h"

// MoveIt Includes
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


// Custom MSG Includes
#include "goal_getter/GoalPose.h"



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

////////  TESTING VARIABLES  /////////
int num_attempts = 0;
int num_max_attempts = 3;
///////////////////////////

// Initialize Hebi Move Group
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
ros::Publisher* speaker_pub_ptr;
ros::Publisher* simulated_joint_states_pub_ptr;

// Sounds
std_msgs::String pushSound;
std_msgs::String pullSound;
std_msgs::String startSound;

// Planning Initializations
int state = 6;
std_msgs::Int32 status;
const std::string PLANNING_GROUP = "dof_4_lowerlonger_arm";
std::string move_group_planner_id = "RRTConnect";

// Initialize Goal and Transform Variables
geometry_msgs::PoseStamped goal_pose;
Eigen::Vector3d goal_pose_normal_vector;
Eigen::Vector3d goal_pose_euler;

std::vector<double> goal_tolerance_pose_default {0.05, 0.05, 0.05};
std::vector<double> goal_tolerance_pose = goal_tolerance_pose_default;

std::vector<double> goal_tolerance_angle_default {10, 0.54, 0.54};

std::vector<double> goal_tolerance_angle = goal_tolerance_angle_default;
double goal_tolerance_pose_adjustment = 0.025;
double goal_tolerance_pose_adjusted_threshold = 0.1;
float goal_offset = 0.07;

std::string moveit_planner = "RRTConnect";
double moveit_planning_time_initial_goal = 2.0;
double moveit_planning_time_return_home = 2.0;
unsigned int num_attempts_initial_goal = 10;
unsigned int num_attempts_stitching = 1;
unsigned int num_attempts_return_home = 5;
planning_interface::MotionPlanRequest plan_req;
planning_interface::MotionPlanResponse plan_res;
moveit_msgs::MotionPlanResponse prev_plan_res;
moveit::planning_interface::MoveItErrorCode moveitSuccess;
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
double plan_execution_start_delay = 0.4;
ros::Time plan_start;
ros::Time plan_end;
std::string end_effector_name = "end_link/INPUT_INTERFACE";

// // // Variables used to turn on or off various features
bool use_stitching = 0;
bool fix_traj_times = 0;
// std::vector<float> max_joint_velocities_RadPerSec = {1, 1, 1, 1};
std::vector<float> max_joint_velocities_RadPerSec = {0.3, 0.3, 0.3, 0.3};
bool use_impedance_over_rr = 0;
Eigen::VectorXd impedance_K_gains(6);
// // // // // // // //
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
std::vector<double> hebiJointAngEfforts(4);
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
std::vector<double> joint_group_positions = { -0.05, -2.0, -2.1, -2.3 };
// std::vector<double> joint_group_positions = { 0.0, 0.0, 0.0, 0.0 };
// std::vector<double> joint_group_positions = { 0.0, -1.8, -2.2, -2.4 };

// Feng Xiang
// Initialize Ready Variables
// std::vector<double> joint_group_ready_position = {1.14, 0.53, -1.54, -1.98};
std::vector<std::vector<double>> joint_group_ready_position;
std::vector<double> joint_group_low_ready_position = {1.1, -0.19, -2.3, -2.40};
std::vector<double> joint_group_mid_ready_position = {1.1, 0.55, -2.3, -2.40};
std::vector<double> joint_group_high_ready_position = {1.1, 1.07, -2.3, -2.40};

bool homeInit = false;
// bool debugBool = false;
// std::vector<std::vector<double>> resolved_rate_joint_limits = {{-2.6, 2.6}, {-2.3, 2.3}, {-2.3, 2.3}, {-2.3, 2.3}};
std::vector<std::vector<double>> resolved_rate_joint_limits = {{-2.6, 2.6}, {-2.3, 2.3}, {-2.0, -0.1}, {-2.4,-0.25}};
bool rr_to_home = false;

// Resolved Rate Globals
double wait_time_for_pushing = 2.0;
double wait_time_for_pulling = 0.5;
double tf_offset_time = 0.05;
double prevGoalCallbackTime = 0.0;
ros::Time prevPoseMotionDetectTime;
// Resolved Rate Global Variables
ros::Time rr_iterate_start_time;
double rr_push_in_distance = 0.035;
double rr_push_in_max = 0.06;
double rr_push_in_min = 0.035;
double rr_iterate_time = 3.0;
double rr_curr_offset = -goal_offset;

bool use_angular_rr = 0;
double position_rr_ratio = 1;
double angular_rr_ratio = 0.0;

bool use_rr_collision_checking = 1;

unsigned int home_plan_attempts = 0;
unsigned int max_home_plan_attempts = 5;
unsigned int home_attempts = 0;
unsigned int max_home_attempts = 5;


geometry_msgs::PoseStamped motorGoalPoseStamped;


// Custom trajectory finish variables
bool manual_trajectory_finish = 0;
std::vector<float> trajectory_finish_position_threshold = {0.03, 0.1, 0.03, 0.04};

// Define Subscriber Callbacks
// Local Goal Update
void goal_callback(const goal_getter::GoalPose::ConstPtr& msg)
{
    // Fill in pose_stamped goal
	goal_pose = msg->goal_normal;
	motorGoalPoseStamped = msg->goal_normal_motor;

	// Calculate normal vector
	Eigen::Quaterniond goal_pose_quaternion(goal_pose.pose.orientation.w, goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z);
	goal_pose_quaternion = goal_pose_quaternion.normalized();
	Eigen::Matrix3d goal_pose_rotation_matrix;
	goal_pose_rotation_matrix = goal_pose_quaternion.toRotationMatrix();
	goal_pose_normal_vector << goal_pose_rotation_matrix(0,0), goal_pose_rotation_matrix(1,0), goal_pose_rotation_matrix(2,0);

	goal_pose_euler = goal_pose_rotation_matrix.eulerAngles(0,1,2);

    if (state == 1)
    {
        state = -1;
    }
    return;
}

void main_cmd_callback(const std_msgs::Int32::ConstPtr& msg)
{
	// NOTE: descriptions need to be updated
	// Functions:
	// 1 = e_stop > Shut off power to motors
	// 2 = hold > Move to and hold plate
	// 3 = compact > Return to home position
	// # = caution > Motors stop moving, but maintain with lower torque threshold

	// Statuses:
    // IDLE = 20 # Waiting for new command
    // INIT = 21 # Starting up node
    // PROCESSING = 22 # Executing command 
    // COMPLETED = 23 # Finished task
    // FAULT = 29 # Error State

	// Old Statuses:
	// 1 = initializing > Command received, but not executing yet(e.g.detecting hands)[ROS INFO]
	// 2 = executing > Command being executed(e.g.moving to target)
	// 3 = waiting > Command completed / performing holding task, ready for next command(maintaining position in 3d space)

	if (msg->data == 1) // go to target
	{
		if (state == 0)
		{
			state = 1;
			status.data = 22; 
			state_input_pub_ptr->publish(status);
			ROS_INFO("Planning to move to target");
		}
	}

	if (msg->data == 2) // go to home
	{
		if (state == 4)
		{
			// Initialize Resolved Rate Variables
			speaker_pub_ptr->publish(pullSound);
			ros::Duration(wait_time_for_pulling).sleep();
			rr_iterate_start_time = ros::Time::now();
			rr_curr_offset = rr_push_in_distance;
			state = 5;
			status.data = 22; 
			state_input_pub_ptr->publish(status);
			ROS_INFO("Leaving target");
			state_input_pub_ptr->publish(status);
		}
		else if (state == 0)
		{
			// state = 6;
			// rr_to_home = true;

			sensor_msgs::JointState hebi_home_msg;

			hebi_home_msg.header.stamp = ros::Time::now();
			hebi_home_msg.name = names;
			hebi_home_msg.header.frame_id = "Position";
			for (unsigned int ii = 0; ii < 4; ii++)
			{
				hebi_home_msg.position.push_back(joint_group_positions[ii]);
			}

			simulated_joint_states_pub_ptr->publish(hebi_home_msg);

		}
	}
}

// MoveIt callback for trajectory finishes
void execute_trajectory_feedback_callback(const moveit_msgs::MoveGroupActionFeedback::ConstPtr& msg)
{
	if (manual_trajectory_finish)
	{
		return;
	}
	std::cout<<"execute_trajectory_feedback_callback called"<<std::endl;
	std::cout<<"msg->feedback.state is: "<<msg->feedback.state<<std::endl;
	std::cout<<"msg->status.text is: "<<msg->status.text<<std::endl;
	// Check if the trajectory has finished
	if (msg->feedback.state == "IDLE" && msg->status.text == "Solution was found and executed.")
	{
		std::cout<<"row::Time::now() is: "<<ros::Time::now()<<std::endl;
		std::cout<<"plan_start is: "<<plan_start<<std::endl;
		// Check if the robot was moving to a target
		if (state == 2)
		{
			speaker_pub_ptr->publish(pushSound);
			// Add a delay so that the user can move their hand
			ros::Duration(wait_time_for_pushing).sleep();

			// Initialize Resolved Rate Variables
			rr_iterate_start_time = ros::Time::now();
			rr_curr_offset = -goal_offset;
			state = 3;
			ROS_INFO("Successfully reached target offset; extending");
		}
		// Check if the robot was returning to home
		else if (state == 6)
		{
			state = 0;
			rr_to_home = false;
			ROS_INFO("Successfully returned to home; waiting");
			// Let the main_state_machine node know that the robot is ready
			status.data = 23; // Old: 3
			state_input_pub_ptr->publish(status);
			status.data = 20;
			state_input_pub_ptr->publish(status);
		}

	}

	else if (msg->feedback.state == "IDLE")
	{
		state = 6;
		rr_to_home = true;
		home_attempts += 1;
		if (home_attempts > max_home_attempts)
		{
			// Send the arm to the ready position through a naive joint move
			sensor_msgs::JointState hebi_home_msg;

			hebi_home_msg.header.stamp = ros::Time::now();
			hebi_home_msg.name = names;
			hebi_home_msg.header.frame_id = "Position";
			for (unsigned int ii = 0; ii < 4; ii++)
			{
				hebi_home_msg.position.push_back(joint_group_ready_position[0][ii]);
			}
			simulated_joint_states_pub_ptr->publish(hebi_home_msg);
			home_attempts = 0;
		}
		/////////////// For FVD we overwrote error status and force go home
		// status.data = 29;
		// state_input_pub_ptr->publish(status);
	}
}

/*------------------------------------------------------------------------
	|  Function HEBI joint angles callback (BLOCK FOR EVERY FUNCTION IN HEADER FILE)

	|  Parameters:
	      parameter_name (IN, OUT, or IN/OUT) -- EXPLANATION OF THE
	              PURPOSE OF THIS PARAMETER TO THE FUNCTION.
	                      (REPEAT THIS FOR ALL FORMAL PARAMETERS OF
	                       THIS FUNCTION.
	                       IN = USED TO PASS DATA INTO THIS FUNCTION,
	                       OUT = USED TO PASS DATA OUT OF THIS FUNCTION
	                       IN/OUT = USED FOR BOTH PURPOSES.)
	
	|  Returns:  IF THIS FUNCTION SENDS BACK A VALUE VIA THE RETURN
	      MECHANISM, DESCRIBE THE PURPOSE OF THAT VALUE HERE.
*-------------------------------------------------------------------*/
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

	hebiJointAngEfforts.at(0) = hebimsg->effort[0];
	hebiJointAngEfforts.at(1) = hebimsg->effort[1];
	hebiJointAngEfforts.at(2) = hebimsg->effort[2];
	hebiJointAngEfforts.at(3) = hebimsg->effort[3];
}

int main(int argc, char** argv)
{	
    // Initialize ROS
	ros::init(argc, argv, "KDC_actuated_manipulation_node");

	// Construct node
	ros::NodeHandle node_handle;
	ros::NodeHandle* node_handle_ptr;
	node_handle_ptr = &node_handle;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	int group_size = names.size();

    //print names of each HEBI item in the group
    std::cout << "Total Number of HEBI Members in group: " << group_size << std::endl;

	// Initialize Local Naive Push Variables
	Eigen::VectorXd joint_velocities(group_size);
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
	robot_state::RobotState& robotCurrState = psm->getCurrentStateNonConst();
	
	std::cout<<"Planning scene initialized"<<std::endl;

	moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
	robot_state_ptr = &robot_state;
	std::cout<<"Robot state set up"<<std::endl;

	/* Create a JointModelGroup to keep track of the current robot pose and planning group. The Joint Model
	group is useful for dealing with one set of joints at a time such as a left arm or a end effector */
	joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

	// Create Planning Pipeline
	planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, *node_handle_ptr, "/Integrated_AM/planning_plugin", "Integrated_AM/request_adapters"));
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
	std::cout<<"Visual tools initialized"<<std::endl;

	// Initialize Publishers
	ros::Publisher state_input_pub = node_handle.advertise<std_msgs::Int32>("/feedback_arm", 5);
	state_input_pub_ptr = &state_input_pub;
	ros::Publisher desired_efforts_pub = node_handle.advertise<sensor_msgs::JointState>("/desired_hebi_efforts", 1);
	ros::Publisher display_publisher = node_handle_ptr->advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	display_publisher_ptr = &display_publisher;
	ros::Publisher speaker_pub = node_handle.advertise<std_msgs::String>("/speaker",5);
	speaker_pub_ptr = &speaker_pub;
	ros::Publisher new_trajectory_pub = node_handle_ptr->advertise<moveit_msgs::MotionPlanResponse>("/new_trajectory", 1, true);


	// Feng Xiang
	// Publisher to publish joint_states for resolved rate controller
	ros::Publisher simulated_joint_states_pub = node_handle.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states",1);
	simulated_joint_states_pub_ptr = &simulated_joint_states_pub;

	// Initialize Subscribers
	ros::Subscriber main_cmd_sub = node_handle.subscribe("/main_cmd", 1, main_cmd_callback);
	ros::Subscriber execute_action_sub = node_handle.subscribe("/execute_trajectory/feedback", 5, execute_trajectory_feedback_callback);
	ros::Subscriber hebi_joints_sub = node_handle.subscribe("/hebi_joints", 1, hebiJointsCallback);
	ros::Subscriber goal_sub = node_handle.subscribe("/goal", 1, goal_callback);

	// Initialize clear_octomap action server
	ros::ServiceClient clear_octo_client = node_handle.serviceClient<std_srvs::Empty>("clear_octomap");
	std_srvs::Empty empty_srv_req;

	// Sounds
	pushSound.data = "pushHumanSound.mp3";
	pullSound.data = "pullHumanSound.mp3";
	startSound.data = "startupSound.mp3";


	// Feng Xiang: hardcoded number of motor joints on robot arm
    Eigen::VectorXd thetas(group_size);
    Eigen::VectorXd thetadot(group_size);

	float singularThresh = 1.0;
	float singularThreshBuffer = 0.25;
    Eigen::MatrixXd W(group_size,group_size);
    W.setIdentity();
	W(0,0) = 1.0;
	W(1,1) = 1.0;
	W(2,2) = 1.0;
	W(3,3) = 1.0;

    prevPoseMotionDetectTime = ros::Time::now();
	joint_group_ready_position.push_back(joint_group_low_ready_position);
	joint_group_ready_position.push_back(joint_group_mid_ready_position);
	joint_group_ready_position.push_back(joint_group_high_ready_position);

	std::cout<<"Publishers and subscribers initialized"<<std::endl;
	speaker_pub_ptr->publish(startSound);

	std::cout<<"Looping and ready"<<std::endl;
	while (ros::ok())
	{
        // Check if the robot has been commanded to move and has received an updated local goal
        if (state == -1)
        {

			// Feng Xiang
			ROS_INFO("Planning to go to ready state.");

			// Create and set planning goal
			robot_state::RobotState goal_state(robot_model);
			goal_state.setJointGroupPositions(joint_model_group, joint_group_ready_position[0]);
			moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
			std::cout<<"joint_goal is: "<<std::endl;
			ROS_INFO_STREAM(joint_goal);

			plan_req.start_state.joint_state.name = prev_plan_res.trajectory.joint_trajectory.joint_names;
			plan_req.start_state.joint_state.position = prev_plan_res.trajectory.joint_trajectory.points[prev_plan_res.trajectory.joint_trajectory.points.size() - 1].positions;
			plan_req.start_state.joint_state.velocity = prev_plan_res.trajectory.joint_trajectory.points[prev_plan_res.trajectory.joint_trajectory.points.size() - 1].velocities;
			plan_req.start_state.joint_state.effort = prev_plan_res.trajectory.joint_trajectory.points[prev_plan_res.trajectory.joint_trajectory.points.size() - 1].effort;

			// Set the start state to the current joint state of the HEBI motors
			std::cout<<"Update the start of the plan request to be the robot's current state"<<std::endl;
			std::cout<<"plan_req.start_state.joint_state.position.size() is: "<<plan_req.start_state.joint_state.position.size()<<std::endl;

			for (unsigned int ii=0; ii < plan_req.start_state.joint_state.position.size(); ii++)
			{
				std::cout<<"ii is: "<<ii<<std::endl;
				std::cout<<"plan_req.start_state.joint_state.position[ii] is: "<<plan_req.start_state.joint_state.position[ii]<<std::endl;
				std::cout<<"hebiJointAngles[ii] is: "<<hebiJointAngles[ii]<<std::endl;
				std::cout<<"hebiJointAngVelocities[ii] is: "<<hebiJointAngVelocities[ii]<<std::endl;

			}

			plan_req.group_name = PLANNING_GROUP;
			plan_req.planner_id = moveit_planner;
			plan_req.num_planning_attempts = num_attempts_return_home;
			plan_req.allowed_planning_time = moveit_planning_time_return_home;
			plan_req.goal_constraints.clear();
			plan_req.goal_constraints.push_back(joint_goal);
			planning_pipeline_global_ptr->generatePlan(*psmPtr, plan_req, plan_res);

			// Store plan
			moveit_msgs::MotionPlanResponse response;
			response_ptr = &response;
			plan_res.getMessage(response);

			// Set the trajectory and execute the plan
			my_plan.trajectory_ = response.trajectory;
			my_plan.start_state_ = response.trajectory_start;
			my_plan.planning_time_ = response.planning_time;

			// Execute move
			move_group.execute(my_plan);
			prev_plan_res = response;
			ROS_INFO("Going to ready state.");

			ros::Duration(1.0).sleep(); //Wait for arm to finish moving?

			// compute orientation goal
			// compute offset
			goal_pose.pose.position.x = goal_pose.pose.position.x - goal_pose_normal_vector(0) * goal_offset;
			goal_pose.pose.position.y = goal_pose.pose.position.y - goal_pose_normal_vector(1) * goal_offset;
			goal_pose.pose.position.z = goal_pose.pose.position.z - goal_pose_normal_vector(2) * goal_offset;

            // Plan RRT Connect Path and send it for execution
            // Set move group planning constraints
			// code block: copy all code from state == -1
            move_group_ptr->setNumPlanningAttempts(num_attempts_initial_goal);
            move_group_ptr->setPlanningTime(moveit_planning_time_initial_goal);

            // Reset tolerances
            goal_tolerance_pose = goal_tolerance_pose_default;
            goal_tolerance_angle = goal_tolerance_angle_default;
            std::cout<<"Calling planning pipeline to generate plan"<<std::endl;
            
			while (true)
            {
                std::cout<<"Adding pose goal"<<std::endl;
                moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(end_effector_name, goal_pose, goal_tolerance_pose, goal_tolerance_angle);
                plan_req.group_name = PLANNING_GROUP;
				plan_req.planner_id = moveit_planner;
				plan_req.num_planning_attempts = num_attempts_return_home;
				plan_req.allowed_planning_time = moveit_planning_time_return_home;
				plan_req.goal_constraints.clear();
                plan_req.goal_constraints.push_back(pose_goal);

                // Update current state to be the last planned state
                std::cout<<"Updating start state"<<std::endl;
                plan_req.start_state.joint_state.name = prev_plan_res.trajectory.joint_trajectory.joint_names;
                plan_req.start_state.joint_state.position = prev_plan_res.trajectory.joint_trajectory.points[prev_plan_res.trajectory.joint_trajectory.points.size() - 1].positions;
                plan_req.start_state.joint_state.velocity = prev_plan_res.trajectory.joint_trajectory.points[prev_plan_res.trajectory.joint_trajectory.points.size() - 1].velocities;
                plan_req.start_state.joint_state.effort = prev_plan_res.trajectory.joint_trajectory.points[prev_plan_res.trajectory.joint_trajectory.points.size() - 1].effort;

				// Set the start state to the current joint state of the HEBI motors
				std::cout<<"Update the start of the plan request to be the robot's current state"<<std::endl;
                std::cout<<"Generating plan, state == -1"<<std::endl;
                planning_pipeline_global_ptr->generatePlan(*psmPtr, plan_req, plan_res);
                if ( sqrt(pow(goal_tolerance_pose[0],2)+pow(goal_tolerance_pose[1],2)+pow(goal_tolerance_pose[2],2)) > goal_tolerance_pose_adjusted_threshold)
                {
                    if (num_attempts < num_max_attempts)
                    {
                        std::cout<<"Couldn't sufficiently plan for point, trying again"<<std::endl;
                        std::cout<<"num_attempts is: "<<num_attempts<<std::endl;
						num_attempts += 1;
                        continue;
                    }
                    state = 6;
                    ROS_INFO("Couldn't find a suitable path, returning to home");

                    // Let the main_state_machine node know that the robot is ready
                    status.data = 29; 
                    state_input_pub_ptr->publish(status);
                    status.data = 20;
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
                if((*psmPtr)->isPathValid(plan_req.start_state, response.trajectory, PLANNING_GROUP))
                {
                    break;
                }
            }

			std::cout<<"num_attempts is: "<<num_attempts<<std::endl;
			std::cout<<"plan_res.error_code_.val is: "<<plan_res.error_code_.val<<std::endl;
			std::cout<<"plan_res.error_code_.SUCCESS is: "<<plan_res.error_code_.SUCCESS<<std::endl;
            num_attempts = 0;

            // Continue through outer while loop
            if (plan_res.error_code_.val != plan_res.error_code_.SUCCESS)
            {
				std::cout<<"plan_res.error_code_.val did not equal plan_res.error_code_.SUCCESS"<<std::endl;
                continue;
            }

            // Visualize the result
            moveit_msgs::DisplayTrajectory display_trajectory;

            // Visualize the trajectory 
            ROS_INFO("Visualizing the trajectory");
            display_trajectory.trajectory_start = response.trajectory_start;
            display_trajectory.trajectory.clear();
            display_trajectory.trajectory.push_back(response.trajectory);
            display_publisher_ptr->publish(display_trajectory);
            visual_tools_ptr->publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
            visual_tools_ptr->trigger();


            // Set the trajectory and execute the plan
            my_plan.trajectory_ = response.trajectory;
			my_plan.start_state_ = response.trajectory_start;
			my_plan.planning_time_ = response.planning_time;

            std::cout<<"my_plan.planning_time_ is: "<<my_plan.planning_time_<<std::endl;
            std::cout<<"my_plan.start_state_ is: "<<my_plan.start_state_<<std::endl;
            std::cout<<"my_plan.trajectory_ is: "<<my_plan.trajectory_<<std::endl;
            std::cout<<"plan_res.planning_time_ is: "<<plan_res.planning_time_<<std::endl;

            prev_plan_res = response;
			plan_start = ros::Time::now();
            state = 2;
            status.data = 22; 
            state_input_pub_ptr->publish(status);
            ROS_INFO("Moving to target");

			move_group_ptr->execute(my_plan);
        }

		if (state == 3 || state == 4 || state == 5)
		{		

			Eigen::VectorXd xg(6);
			xg << goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z, goal_pose_euler(0), goal_pose_euler(1), goal_pose_euler(2);

			for (unsigned int ii = 0; ii < group_size; ii++)
			{
				thetas(ii) = hebiJointAngles.at(ii);
			}

			std::vector<double> joint_values{thetas(0), thetas(1), thetas(2), thetas(3)};
			robotCurrState.setJointGroupPositions(joint_model_group, joint_values);
			const Eigen::Affine3d& link_pose = robotCurrState.getGlobalLinkTransform("end_link/INPUT_INTERFACE");

			Eigen::VectorXd x0(6);
			Eigen::Vector3d x0cart = link_pose.translation();
			Eigen::Vector3d x0euler = link_pose.rotation().eulerAngles(0,1,2);
			x0 << x0cart(0), x0cart(1), x0cart(2), x0euler(0), x0euler(1), x0euler(2);

            Eigen::MatrixXd J;
			Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
			robotCurrState.getJacobian(joint_model_group, robotCurrState.getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, J);
			Eigen::MatrixXd ee_J_cartesian = J.block(0,0,3,group_size);
			Eigen::MatrixXd ee_J_angular = J.block(3,0,3,group_size);

			// Adjust push in distance based on the goal's orientation (more force if point up, less if pointed horizontally)
			rr_push_in_distance = abs(goal_pose_normal_vector(2)) * (rr_push_in_max - rr_push_in_min) + rr_push_in_min;

			// Iterate goal forward or backwards
			if (state == 3)
			{
				rr_curr_offset = ((ros::Time::now() - rr_iterate_start_time).toSec() / rr_iterate_time) * (goal_offset + rr_push_in_distance) - goal_offset;
				if (rr_curr_offset >= rr_push_in_distance)
				{
					rr_curr_offset = rr_push_in_distance;
					state = 4;
				}
			}
			else if (state == 5)
			{
				rr_curr_offset = rr_push_in_distance - ((ros::Time::now() - rr_iterate_start_time).toSec() / rr_iterate_time) * (goal_offset + rr_push_in_distance);
				if (rr_curr_offset <= -goal_offset)
				{
					rr_curr_offset = -goal_offset;
					state = 6;
				}
			}
			Eigen::Vector3d curr_goal_offset;
			curr_goal_offset = rr_curr_offset * goal_pose_normal_vector;

			// err of position to goal
			Eigen::VectorXd err(6);
            // err = xg + curr_goal_offset - x0;
			err = xg - x0;
			err(0) += curr_goal_offset(0);
			err(1) += curr_goal_offset(1);
			err(2) += curr_goal_offset(2);
			
			Eigen::VectorXd err_cartesian(3);
			Eigen::Vector3d xg_cartesian;
			xg_cartesian << xg(0), xg(1), xg(2);
			Eigen::VectorXd err_angular(3);
			Eigen::Vector3d xg_angular;
			xg_angular << xg(3), xg(4), xg(5);
			err_cartesian = xg_cartesian + curr_goal_offset - x0cart;
			err_angular = xg_angular - x0euler;

			if (use_angular_rr)
			{
				thetadot = position_rr_ratio * (W.inverse()*ee_J_cartesian.transpose()*(ee_J_cartesian*W.inverse()*ee_J_cartesian.transpose()).inverse()*err_cartesian);
				std::cout<<"RR without angular is: "<<thetadot<<std::endl;
				thetadot = thetadot + angular_rr_ratio * (W.inverse()*ee_J_angular.transpose()*(ee_J_angular*W.inverse()*ee_J_angular.transpose()).inverse()*err_angular);
				std::cout<<"RR with angular is: "<<thetadot<<std::endl;
			}
			else
			{
				if (W.isIdentity(0.1))
				{
					thetadot = ee_J_cartesian.transpose()*(ee_J_cartesian*ee_J_cartesian.transpose()).inverse()*err_cartesian;
				}
				else
				{
					thetadot = W.inverse()*ee_J_cartesian.transpose()*(ee_J_cartesian*W.inverse()*ee_J_cartesian.transpose()).inverse()*err_cartesian;
				}
			}

			Eigen::VectorXd tempThetas = thetas+thetadot;
			if(std::abs(tempThetas(2)) < singularThresh && std::abs(tempThetas(3)) < singularThresh){
				std::cout << "[RR] Arm at Singularity - " << abs(goal_pose_normal_vector(2)) << " so gain is " << (5-4*abs(goal_pose_normal_vector(2))) << std::endl;
				W(0,0) = (5-4*abs(goal_pose_normal_vector(2)));
				W(1,1) = (5-4*abs(goal_pose_normal_vector(2)));
			}
			else
			{
				W(0,0) = 1;
				W(1,1) = 1;
			}
			thetas = thetas + thetadot;

			for (unsigned int ii = 0; ii < group_size; ii++)
			{
				if (thetas(ii) < resolved_rate_joint_limits[ii][0])
				{
					thetas(ii) = resolved_rate_joint_limits[ii][0];
				}
				else if (thetas(ii) > resolved_rate_joint_limits[ii][1])
				{
					thetas(ii) = resolved_rate_joint_limits[ii][1];
				}
			}

			sensor_msgs::JointState hebi_thetas_msg;

			hebi_thetas_msg.header.stamp = ros::Time::now();
			hebi_thetas_msg.header.frame_id = "Position";
			hebi_thetas_msg.name = names;

			if (use_rr_collision_checking)
			{
				// Set the joint positions of the robot state to the proposed joint positions
				std::vector<double> theta_vector;
				for (unsigned int ii = 0; ii < group_size; ii++)
				{
					theta_vector.push_back(thetas(ii));
				}
				robot_state::RobotState& current_state = psm->getCurrentStateNonConst();
				current_state.setVariablePositions(theta_vector);
				collision_detection::CollisionRequest c_req;
				collision_detection::CollisionResult c_res;
				psm->checkSelfCollision(c_req, c_res);

				if (!c_res.collision)
				{
					std::cout<<"No collision detected"<<std::endl;
					for (unsigned int ii = 0; ii < group_size; ii++)
					{
						hebi_thetas_msg.position.push_back(thetas(ii));
					}
					simulated_joint_states_pub.publish(hebi_thetas_msg);
				}
				else
				{
					std::cout<<"Collision detected"<<std::endl;
				}
			}
			else
			{
				for (unsigned int ii = 0; ii < group_size; ii++)
				{
					hebi_thetas_msg.position.push_back(thetas(ii));
				}
				simulated_joint_states_pub.publish(hebi_thetas_msg);
			}
			if (rr_to_home == false)
			{
				rr_to_home = true;
			}
		}

		// Check if the robot is returning to home
		if (state == 6)
		{
			robot_state::RobotState goal_state(robot_model);
			if (homeInit == false)
			{
				plan_req.start_state.joint_state.name = {"motor1/X5_9", "motor2/X8_16", "motor3/X5_9", "motor4/X5_4"};
				plan_req.start_state.joint_state.position = {0.0, -1.93, -2.38, -2.50};
				plan_req.start_state.joint_state.velocity = {};
				plan_req.start_state.joint_state.effort = {};
				goal_state.setJointGroupPositions(joint_model_group, joint_group_positions);
				clear_octo_client.call(empty_srv_req);

				plan_req.goal_constraints.clear();
				moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
				plan_req.goal_constraints.push_back(joint_goal);
				std::cout<<"joint_goal is: "<<std::endl;
				ROS_INFO_STREAM(joint_goal);

				ROS_INFO("Planning to return home");
				// Set move group planning constraints
				move_group.setNumPlanningAttempts(num_attempts_return_home);
				move_group.setPlanningTime(moveit_planning_time_return_home);
				
				// Create and set planning goal
				clear_octo_client.call(empty_srv_req);
				plan_req.group_name = PLANNING_GROUP;
				plan_req.planner_id = moveit_planner;
				plan_req.num_planning_attempts = num_attempts_return_home;
				plan_req.allowed_planning_time = moveit_planning_time_return_home;
				planning_pipeline_global_ptr->generatePlan(*psmPtr, plan_req, plan_res);
				
				if (plan_res.error_code_.val != plan_res.error_code_.SUCCESS)
				{

					continue;
				}
				homeInit = true;
			}
			else if (rr_to_home == true)
			{
				plan_req.start_state.joint_state.name = prev_plan_res.trajectory.joint_trajectory.joint_names;
				plan_req.start_state.joint_state.position = prev_plan_res.trajectory.joint_trajectory.points[prev_plan_res.trajectory.joint_trajectory.points.size() - 1].positions;
				plan_req.start_state.joint_state.velocity = prev_plan_res.trajectory.joint_trajectory.points[prev_plan_res.trajectory.joint_trajectory.points.size() - 1].velocities;
				plan_req.start_state.joint_state.effort = prev_plan_res.trajectory.joint_trajectory.points[prev_plan_res.trajectory.joint_trajectory.points.size() - 1].effort;

				plan_req.goal_constraints.clear();
				goal_state.setJointGroupPositions(joint_model_group, joint_group_ready_position[0]);

				moveit_msgs::Constraints joint_goal_0 = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
				plan_req.goal_constraints.push_back(joint_goal_0);
				std::cout<<"joint_goal_0 is: "<<std::endl;
				ROS_INFO_STREAM(joint_goal_0);
			
				goal_state.setJointGroupPositions(joint_model_group, joint_group_positions);

				moveit_msgs::Constraints joint_goal_1 = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
				plan_req.goal_constraints.push_back(joint_goal_1);
				std::cout<<"joint_goal_1 is: "<<std::endl;
				ROS_INFO_STREAM(joint_goal_1);

				ROS_INFO("Planning to return home");
				// Set move group planning constraints
				move_group.setNumPlanningAttempts(num_attempts_return_home);
				move_group.setPlanningTime(moveit_planning_time_return_home);
				
				// Create and set planning goal
				clear_octo_client.call(empty_srv_req);
				plan_req.group_name = PLANNING_GROUP;
				plan_req.planner_id = moveit_planner;
				plan_req.num_planning_attempts = num_attempts_return_home;
				plan_req.allowed_planning_time = moveit_planning_time_return_home;
				planning_pipeline_global_ptr->generatePlan(*psmPtr, plan_req, plan_res);
				if (plan_res.error_code_.val != plan_res.error_code_.SUCCESS)
				{
					std::cout << "rr_to_home plan failed. planning again." << std::endl;
					continue;
				}
				
			}

			// Let the main_state_machine node know that the robot is initializing
			status.data = 22;
			state_input_pub.publish(status);
			
			// Store plan
			moveit_msgs::MotionPlanResponse response;
			plan_res.getMessage(response);
			response_ptr = &response;
			std::cout<<"response is: "<<response<<std::endl;

			//Visualize the result
            moveit_msgs::DisplayTrajectory display_trajectory;

            // Visualize the trajectory
            ROS_INFO("Visualizing the trajectory");
			display_trajectory.trajectory.clear();
            display_trajectory.trajectory_start = response.trajectory_start;
            display_trajectory.trajectory.push_back(response.trajectory);
            display_publisher_ptr->publish(display_trajectory);
            visual_tools_ptr->publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
            visual_tools_ptr->trigger();

			// Set the trajectory and execute the plan
			my_plan.trajectory_ = response.trajectory;
			my_plan.start_state_ = response.trajectory_start;
			my_plan.planning_time_ = response.planning_time;

			std::cout << "executing home plan to return to home." << std::endl;
			move_group_ptr->execute(my_plan);
			prev_plan_res = response;
			std::cout << "home plan executed. waahahooooo." << std::endl;
		}

		// Manually check if the robot got to its target
		if ((state == 2 || state == 6) && manual_trajectory_finish)
		{
			std::cout<<"Manually checking trajectory finish"<<std::endl;
			unsigned int state_solved = 1;
			float joint_difference;
			std::cout<<"Variables initialized"<<std::endl;
			for (unsigned int ii = 0; ii < hebiJointAngles.size(); ii++)
			// for(unsigned int ii : indices(hebiJointAngles))
			{
				std::cout<<"ii is: "<<ii<<std::endl;
				std::cout<<"prev_plan_res.trajectory.joint_trajectory.points[prev_plan_res.trajectory.joint_trajectory.points.size() - 1].positions[ii] is: "<<prev_plan_res.trajectory.joint_trajectory.points[prev_plan_res.trajectory.joint_trajectory.points.size() - 1].positions[ii]<<std::endl;
				std::cout<<"hebiJointAngles[ii] is: "<<hebiJointAngles[ii];
				joint_difference = prev_plan_res.trajectory.joint_trajectory.points[prev_plan_res.trajectory.joint_trajectory.points.size() - 1].positions[ii] - hebiJointAngles[ii];
				std::cout<<"joint_difference is: "<<joint_difference<<std::endl;
    			if (abs(joint_difference) > trajectory_finish_position_threshold[ii])
				{
					std::cout<<"State unsolved, position"<<std::endl;
					state_solved = 0;
					break;
				}
			}
			if (state_solved == 1)
			{
				std::cout<<"Trajectory finished!"<<std::endl;
				if (state == 2)
				{
					state = 3;
					// Initialize Resolved Rate Variables
					rr_iterate_start_time = ros::Time::now();
					rr_curr_offset = -goal_offset;
					std::cout<<"Successfully reached offset target, extending"<<std::endl;
				}
				else if (state == 6)
				{
					state = 0;
					ROS_INFO("Successfully returned to home; waiting");
					// Let the main_state_machine node know that the robot is ready
					status.data = 23; // Old: 3
					state_input_pub_ptr->publish(status);
					status.data = 20;
					state_input_pub_ptr->publish(status);
				}
			}
		}
	}
	// Shutdown alongside ROS
	return 0;
}
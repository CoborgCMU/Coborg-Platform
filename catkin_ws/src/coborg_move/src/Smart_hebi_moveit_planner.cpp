#include <stdio.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include "hebi_cpp_api/trajectory.hpp"
#include "hebi_cpp_api/robot_model.hpp"


#include "Eigen/Eigen"


#include <algorithm> //based on HEBI C++ API 3.3.0 documentation
#include <cmath>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <pluginlib/class_loader.h>
#include <boost/scoped_ptr.hpp>

#include <sensor_msgs/JointState.h>

#include <string.h>

// declare motor joints as global vairables
std::string control_type;
double motor1_joint;
double motor2_joint;
double motor3_joint;
double motor4_joint;

// start up variables
// set start up procedure to prevent max torque at start up phenomena
bool boolFirstTime = false;
double startup_sec = 0.1;

// set global variable to publish joint angles
// geometry_msgs::Twist publishState;
sensor_msgs::JointState torqueVect;

ros::Time currImp;

bool impValue = false;


// convert joint angles from MoveIt node to HEBI joint angles
void hebiOutputCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // ROS_INFO("motor1: %f | motor2: %f | motor3: %f", msg->position[0],msg->position[1],msg->position[2]);
    std::cout<<"hebiOutputCallback received"<<std::endl; // // //
    control_type = msg->header.frame_id;
    motor1_joint = msg->position[0]; // offset determined empirically for level arm out at 0 radians 
    motor2_joint = msg->position[1];
    motor3_joint = msg->position[2];
    motor4_joint = msg->position[3];
    std::cout<<"motor joint positions filled"<<std::endl; // // //
}

void effortCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std::cout<<"effortCallback received"<<std::endl; // // //
    impValue = true;
    currImp = ros::Time::now();
    ROS_INFO_STREAM("Received Torque Command : \n" << *msg << "\n");
    torqueVect = *msg;
    std::cout<<"effortCallback finalized"<<std::endl; // // //
}

Eigen::VectorXd getGravityCompensationEfforts(const hebi::robot_model::RobotModel& model, const Eigen::VectorXd& masses, const Eigen::VectorXd& positions, const Eigen::Vector3d& gravity) {
    std::cout << "starting gravity compensation" << std::endl;
    // Normalize gravity vector (to 1g, or 9.8 m/s^2)
    Eigen::Vector3d normed_gravity = gravity;
    normed_gravity /= normed_gravity.norm();
    normed_gravity *= 9.81;

    size_t num_dof = model.getDoFCount();
    size_t num_frames = model.getFrameCount(hebi::robot_model::FrameType::CenterOfMass);

    hebi::robot_model::MatrixXdVector jacobians;
    model.getJ(hebi::robot_model::FrameType::CenterOfMass, positions, jacobians);

    // Get torque for each module
    // comp_torque = J' * wrench_vector
    // (for each frame, sum this quantity)
    Eigen::VectorXd comp_torque(num_dof);
    comp_torque.setZero();

    // Wrench vector
    Eigen::VectorXd wrench_vec(6); // For a single frame; this is (Fx/y/z, tau x/y/z)
    wrench_vec.setZero();

    for (size_t i = 0; i < num_frames; i++){
        jacobians[i](2,0) *= -1;
        jacobians[i](2,1) *= -1;
        jacobians[i](2,2) *= -1;
        jacobians[i](2,3) *= -1;
        jacobians[i](4,0) *= -1;
        jacobians[i](4,1) *= -1;
        jacobians[i](4,2) *= -1;
        jacobians[i](4,3) *= -1;
    }


    for (size_t i = 0; i < num_frames; i++) {
        // Set translational part
        for (size_t j = 0; j < 3; j++) {
            wrench_vec[j] = -normed_gravity[j] * masses[i];
        }

        // Add the torques for each joint to support the mass at this frame
        std::cout << "Number of Frames: " << num_frames << std::endl;
        std::cout << "Jacobian Number: " << i << " | Jacobian: " << jacobians[i] << std::endl;
        

        comp_torque += jacobians[i].transpose() * wrench_vec;
    }

    return comp_torque;
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "KDC_find_hebi_moveit_planner");
    ros::NodeHandle node;

    //Goals
    //Subscribe to /move_group/fake_controller_states rostopic
    //output the topic to the screen
    ros::Subscriber fake_joint_states_sub = node.subscribe("/move_group/fake_controller_joint_states", 1, hebiOutputCallback);
    // ros::Publisher hebi_joints_pub = node.advertise<geometry_msgs::Twist>("hebi_joints", 1);
    ros::Publisher hebi_jointstate_pub = node.advertise<sensor_msgs::JointState>("hebi_joints", 1);

    ros::Subscriber desired_torques_sub = node.subscribe("/desired_hebi_efforts", 1, effortCallback);

    //TODO: create method to auto find the families and names
    //FORNOW: assume there is 1 family and 3 names
    std::vector<std::string> families;
    families = {"01-base","02-shoulder","03-elbow","04-wrist"};

    std::vector<std::string> names;
    names = {"base_1", "shoulder_2", "elbow_3", "wrist_4"};



    // connect to HEBI joints on network through UDP connection
    std::shared_ptr<hebi::Group> group;
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
    //code stolen from hebi_cpp_api_examples/src/basic/group_node.cpp

    // error out if HEBI joints are not found on the network
    if (!group) {
        ROS_ERROR("Could not initialize arm! Check for modules on the network, and ensure good connection (e.g., check packet loss plot in Scope). Shutting down...");
        return -1;
    }
    //print names of each HEBI item in the group
    std::cout << "Total Number of HEBI Members in group: " << group->size() << std::endl;

    // set HEBI command variables
    hebi::GroupCommand groupCommandBegin(group->size());
    
    hebi::GroupCommand groupCommandStabilize(group->size());

    // set positions based on motor joints angles
    // VectorXd will get passed into HEBI joint command
    Eigen::VectorXd positions(group->size());
    Eigen::VectorXd torques(group->size());

    //define feedback variables
    hebi::GroupFeedback group_feedback(group->size());
    Eigen::VectorXd feedbackPos(group->size());
    Eigen::VectorXd feedbackVel(group->size());
    Eigen::VectorXd feedbackTor(group->size());
    feedbackPos.setZero();

    // Gravity compensation variables
    double gravity = 0.98;
    Eigen::VectorXd masses(group->size());
    Eigen::VectorXd link_lengths(group->size());
    Eigen::VectorXd center_of_masses(group->size());
    masses << 0, 0, 0, 0;
    link_lengths << 0.1524, 0.254, 0.2286, 0.2794;
    center_of_masses << 0.1, 0.2, 0.2, 0.2;

    // (impedance control) declare varaibles to be using for force control state
    group->setFeedbackFrequencyHz(20);
    ros::Time begin = ros::Time::now();
    ros::Time curr = ros::Time::now();
    ros::Time beginImp = ros::Time::now();
    double durr;
    Eigen::VectorXd startupVelocity(group->size());
    startupVelocity.setOnes();
    const double stiffness = 1.0;

    std::string cwd("\0", FILENAME_MAX+1);
    std::cout << "Current path: " << getcwd(&cwd[0],cwd.capacity()) << std::endl;
    std::unique_ptr<hebi::robot_model::RobotModel> model = hebi::robot_model::RobotModel::loadHRDF("dof_4_robot.hrdf");
    Eigen::Matrix4d baseTransform;
    baseTransform.setIdentity();
    model->setBaseFrame(baseTransform);
	if (model == NULL)
    {
        ROS_WARN("[RESOLVED RATE] Initialization - Did NOT find HRDF file of robot arm.");
    }
    else
    {
        std::cout << "[RESOLVED RATE] Initialization - Found HRDF file of robot arm." << std::endl;
    }

    double force_time_offset = 5.0;
    double force_ref_time = ros::Time::now().toSec();

    ros::Rate loop_rate(20.0);

    while (ros::ok()) {
        beginImp = ros::Time::now();
        std::cout<<"Time recorded"<<std::endl; // // //


        // compute duration variable
        durr = (float)(curr - begin).toSec();
        std::cout<<"Duration computed"<<std::endl; // // //

        hebi::GroupCommand groupCommand(group->size());


        if (ros::Time::now().toSec() - force_ref_time < force_time_offset)
        {
            control_type = "Position";
            motor1_joint = 0;
            motor2_joint = 0;
            motor3_joint = 0;
            motor4_joint = 0;
        }
        else
        {
            control_type = "Impedance";
            motor1_joint = 0;
            motor2_joint = 0;
            motor3_joint = 0;
            motor4_joint = 0;
        }
        std::cout << control_type << std::endl;

        // if the motors sendback feedback information
        if (group->getNextFeedback(group_feedback))
        {
            std::cout<<"Feedback information received"<<std::endl; // // //
            feedbackPos = group_feedback.getPosition();
            feedbackVel = group_feedback.getVelocity();
            feedbackTor = group_feedback.getEffort();
            std::cout<<"Feedback information recorded"<<std::endl; // // //

            sensor_msgs::JointState hebi_feedback_message;
            hebi_feedback_message.header.stamp = ros::Time::now();
            std::cout<<"Feedback timestamp recorded"<<std::endl; // // //

            for (unsigned int it = 0; it < names.size(); it++)
            {
                hebi_feedback_message.name.push_back(names[it]);

                hebi_feedback_message.position.push_back((float) feedbackPos(it));
                hebi_feedback_message.velocity.push_back((float) feedbackVel(it));
                hebi_feedback_message.effort.push_back((float) feedbackTor(it));
                std::cout<<"Pushed back hebi feedback info"<<std::endl; // // //

            }

            hebi_jointstate_pub.publish(hebi_feedback_message);

            // intiliaze if HEBI motors are starting up for the first time
            if (boolFirstTime)
            {

                // TODO: load xml gain files that has the velocity and effort limits on them
                // FORNOW: hardcode velocity numbers and push those velocities to the joints for a period of time
                curr = ros::Time::now();
                startupVelocity[0] = 0.2 * durr * (motor1_joint - hebi_feedback_message.position[0]);
                startupVelocity[1] = 0.2 * durr * (motor2_joint - hebi_feedback_message.position[1]);
                startupVelocity[2] = 0.2 * durr * (motor3_joint - hebi_feedback_message.position[2]);
                startupVelocity[3] = 0.2 * durr * (motor4_joint - hebi_feedback_message.position[3]);

                groupCommandBegin.setVelocity(startupVelocity);
                group->sendCommand(groupCommandBegin);

                if (durr > startup_sec)
                {
                    boolFirstTime = false;
                }
            }
            else
            {
                curr = ros::Time::now();
                // position control
                positions[0] = motor1_joint;
                positions[1] = motor2_joint;
                positions[2] = motor3_joint;
                positions[3] = motor4_joint;
                if (control_type == "Impedance")
                {
                    // Add gravity compensation
                    
                    // double gravity = 0.98;
                    // Eigen::VectorXd masses(group_size);
                    // Eigen::VectorXd link_lengths(group_size);
                    // Eigen::VectorXd center_of_masses(group_size);
                    // masses << 0, 0, 0, 0;
                    // link_lengths << 0.1524, 0.254, 0.2286, 0.2794;
                    // center_of_masses << 0.1, 0.2, 0.2, 0.2;

                    // Eigen::VectorXd masses(model->getFrameCount(HebiFrameTypeCenterOfMass));
                    // model->getMasses(masses);
                    double bracket_weight_kg = .076;
                    double link_density_kgpm = .429714;
                    double end_effector_kg = .22414;
                    double mass_x5_9_kg = 0.36;
                    double mass_x8_16_kg = 0.5;
                    double mass_x5_4_kg = 0.335;

                    Eigen::VectorXd masses(9);
                    // masses << mass_x5_9_kg, bracket_weight_kg * 2 + link_density_kgpm * .127, mass_x8_16_kg, bracket_weight_kg * 2 + link_density_kgpm * 0.2286, mass_x5_9_kg, bracket_weight_kg * 2 + link_density_kgpm * 0.1905, mass_x5_4_kg, end_effector_kg, 0;
                    masses << mass_x5_9_kg, bracket_weight_kg * 2 + link_density_kgpm * .127, mass_x8_16_kg * 0.2, bracket_weight_kg * 2 + link_density_kgpm * 0.2286, mass_x5_9_kg * 2.7, bracket_weight_kg * 2 + link_density_kgpm * 0.1905, mass_x5_4_kg * 0.7, end_effector_kg * 0.01, 0;

                    auto base_accel = group_feedback[0].imu().accelerometer().get();
                    // Vector3d gravity(-base_accel.getX(), -base_accel.getY(), -base_accel.getZ());
                    Eigen::Vector3d gravity(0, 0, -1);

                    Eigen::VectorXd effort(group->size());
                    effort = getGravityCompensationEfforts(*model, masses, feedbackPos, gravity);
                    
                    double effort_comp = 1.0;
                    // ros::console::shutdown();
				    // std::cout.clear();
	    			// std::cout<<"/////////////////////////////////"<<std::endl;
                    // std::cout<<"masses is: "<<masses<<std::endl;
                    // std::cout<<"feedbackPos is: "<<feedbackPos<<std::endl;
                    // std::cout<<"efforts is: "<<effort<<std::endl;
                    // std::cout<<"current effort is: " << feedbackTor << std::endl;
                    positions[0] = effort(0);
                    positions[1] = effort(1);
                    positions[2] = effort(2);
                    positions[3] = effort(3);
                    positions *= effort_comp;
                    groupCommand.setEffort(positions);
    				// std::cout.setstate(std::ios_base::failbit);
                }
                else
                {
                    groupCommand.setPosition(positions);
                }

                group->sendCommand(groupCommand);
                
                // ROS_INFO_STREAM((float) durr);
                impValue = false;

            }



        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
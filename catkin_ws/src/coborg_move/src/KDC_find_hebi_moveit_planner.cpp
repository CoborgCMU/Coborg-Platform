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

// declare motor joints as global vairables -- Initial values are tucked position
double motor1_joint = -0.05;
double motor2_joint = -2.0;
double motor3_joint = -2.1;
double motor4_joint = -2.3;

// start up variables
// set start up procedure to prevent max torque at start up phenomena
bool boolFirstTime = true;
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
    std::vector<std::string> ros_names;
    ros_names = {"motor1/X5_9", "motor2/X8_16", "motor3/X5_9", "motor4/X5_4"};

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
    hebi::GroupCommand groupCommand(group->size());
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

    // (impedance control) declare varaibles to be using for force control state
    group->setFeedbackFrequencyHz(200);
    ros::Time begin = ros::Time::now();
    ros::Time curr = ros::Time::now();
    ros::Time beginImp = ros::Time::now();
    double durr;
    Eigen::VectorXd startupVelocity(group->size());
    startupVelocity.setOnes();
    const double stiffness = 1.0;

    

    ros::Rate loop_rate(20.0);

    while (ros::ok()) {
        beginImp = ros::Time::now();
        std::cout<<"Time recorded"<<std::endl; // // //


        // compute duration variable
        durr = (float)(curr - begin).toSec();
        std::cout<<"Duration computed"<<std::endl; // // //

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
                hebi_feedback_message.name.push_back(ros_names[it]);

                hebi_feedback_message.position.push_back((float) feedbackPos(it));
                hebi_feedback_message.velocity.push_back((float) feedbackVel(it));
                hebi_feedback_message.effort.push_back((float) feedbackTor(it));
                std::cout<<"Pushed back hebi feedback info"<<std::endl; // // //

            }

            hebi_jointstate_pub.publish(hebi_feedback_message);

            curr = ros::Time::now();
            // position control
            positions[0] = motor1_joint;
            positions[1] = motor2_joint;
            positions[2] = motor3_joint;
            positions[3] = motor4_joint;

            // velocity

            groupCommand.setPosition(positions);
            // groupCommand.setVelocity(velocities);

            // group->sendCommand(groupCommand);
            
            group->sendCommand(groupCommand);
            // ROS_INFO_STREAM((float) durr);
            impValue = false;




        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
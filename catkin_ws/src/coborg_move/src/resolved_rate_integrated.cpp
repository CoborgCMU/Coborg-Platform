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

#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>

#include "Eigen/Eigen"

#include <algorithm> //based on HEBI C++ API 3.3.0 documentation
#include <cmath>
#include <math.h>

#include "std_msgs/Bool.h"
#include <gb_visual_detection_3d_msgs/goal_msg.h>
#include <goal_getter/GoalPose.h>

#include <unistd.h>
#include <iostream>

tf::StampedTransform* prevTransform;
tf::TransformListener* globalListener;
gb_visual_detection_3d_msgs::goal_msg goal; 
bool enable_rr;
double tf_offset_time = 0.05;
double prevGoalCallbackTime = 0.0;
ros::Time prevPoseMotionDetectTime;

geometry_msgs::PoseStamped motorGoalPoseStamped;
geometry_msgs::Pose globalGoalPose;

// geometry_msgs::Pose poseMotionDetection(const geometry_msgs::PoseStamped& pose_msg, const geometry_msgs::Pose& old_pose_msg)
// {    
    

//     if (ros::Time::now().toSec() - prevPoseMotionDetectTime.toSec() > tf_offset_time)
//     {
//         try
//         {

//             geometry_msgs::PoseStamped tempMotor1Pose;
//             globalListener->transformPose("/motor1/INPUT_INTERFACE", pose_msg, tempMotor1Pose);
//             // tempMotor1Pose = pose_msg;
            
//             prevPoseMotionDetectTime = ros::Time::now();

//             geometry_msgs::Pose target_pose;
//             target_pose.position.x = tempMotor1Pose.pose.position.x;
//             target_pose.position.y = tempMotor1Pose.pose.position.y;
//             target_pose.position.z = tempMotor1Pose.pose.position.z;
//             target_pose.orientation.x = tempMotor1Pose.pose.orientation.x;
//             target_pose.orientation.y = tempMotor1Pose.pose.orientation.y;
//             target_pose.orientation.z = tempMotor1Pose.pose.orientation.z;
//             target_pose.orientation.w =  tempMotor1Pose.pose.orientation.w;

//             // ROS_INFO("Motor1 Transform are: x: %f, y: %f: z: %f", target_pose.position.x, target_pose.position.y, target_pose.position.z);

//             return target_pose;
//         }
//         catch (tf::TransformException &ex)
//         {
//             ROS_ERROR("Pose Transform Error: %s", ex.what());
//             return old_pose_msg;
//         }
//     }
//     else
//     {
//         return old_pose_msg;
//     }

// }

void enable_rr_callback(const std_msgs::Bool::ConstPtr& bool_msg)
{
    enable_rr = bool_msg->data;
}

void goal_pose_callback(const goal_getter::GoalPose::ConstPtr& goal_msg)
{
    ROS_INFO("Goal Pose Received.");
    motorGoalPoseStamped = goal_msg->goal_normal_motor;
    if (enable_rr == false)
    {
        enable_rr = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "resolved_rate_planner");
    ros::NodeHandle node;

    // Establish initial values
    tf::TransformListener listener;
    globalListener = &listener;
    

    enable_rr = false;

    
    // Get goal position once it's published by vision nodes (same goal move_it picks up)
    // ros::Subscriber goal_pos_sub = node.subscribe("/cam2_goal",1,goal_callback);
    // Topic to trigger resolved rate to begin
    ros::Subscriber tigger_rr_sub = node.subscribe("/enable_rr",1,enable_rr_callback);
    ros::Subscriber goal_pose_sub = node.subscribe("/goal",1,goal_pose_callback);


    // inititalize HEBI API
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

        for(size_t hebi_iter = 0; hebi_iter < entry_list->size(); ++ hebi_iter)
        {
            std::cout << (*entry_list)[hebi_iter].family_ << " | " << (*entry_list)[hebi_iter].name_ << std::endl;
        }
        break;
      }
      ROS_WARN("[RESOLVED RATE] Initialization - Could not find group actuators, trying again...");
      ros::Duration(1.0).sleep();
    }
    //code stolen from hebi_cpp_api_examples/src/basic/group_node.cpp

    // error out if HEBI joints are not found on the network
    if (!group) {
      ROS_ERROR("[RESOLVED RATE] Initialization - Could not initialize arm! Check for modules on the network, and ensure good connection (e.g., check packet loss plot in Scope). Shutting down...");
      return -1;
    }
    //print names of each HEBI item in the group
    std::cout << "Total Number of HEBI Members in group: " << group->size() << std::endl;

    hebi::GroupCommand groupCommand(group->size());

    hebi::GroupFeedback group_feedback(group->size());
    Eigen::VectorXd thetas(group->size());
    Eigen::VectorXd thetadot(group->size());

    float dt = 1.0;
    Eigen::MatrixXd W(group->size(),group->size());
    W.setIdentity();
    // W(3,3) = 10;

    std::string cwd("\0", FILENAME_MAX+1);
    std::cout << "Current path: " << getcwd(&cwd[0],cwd.capacity()) << std::endl;

    std::unique_ptr<hebi::robot_model::RobotModel> model = hebi::robot_model::RobotModel::loadHRDF("dof_4_robot.hrdf");
    Eigen::Matrix4d baseTransform;
    baseTransform.setIdentity();
    // baseTransform(2,2) = -1;
    model->setBaseFrame(baseTransform);

    if (model == NULL)
    {
        ROS_WARN("[RESOLVED RATE] Initialization - Did NOT find HRDF file of robot arm.");

    }
    else
    {
        
        std::cout << "[RESOLVED RATE] Initialization - Found HRDF file of robot arm." << std::endl;
    }

    prevPoseMotionDetectTime = ros::Time::now();

    ros::Rate rate(10.0);
    while(ros::ok())
    {
        // goal_pub.publish(test_goal);
        
        if(enable_rr)
        {
            // xg = goal
            // convert PoseStamped msg to vectorXd

            // globalGoalPose = poseMotionDetection(motorGoalPoseStamped, globalGoalPose);

            // tf::Quaternion tempQuat(globalGoalPose.orientation.x, globalGoalPose.orientation.y, globalGoalPose.orientation.z, globalGoalPose.orientation.w);
            // double rollg;
            // double pitchg;
            // double yawg;
            // tf::Matrix3x3 tempMat(tempQuat);
            // tempMat.getRPY(rollg, pitchg, yawg);

            // Feng Xiang
            // Eigen::VectorXd xg(6);
            // xg << globalGoalPose.position.x, globalGoalPose.position.y, globalGoalPose.position.z, rollg, pitchg, yawg; 

            Eigen::Vector3d xg;
            xg << motorGoalPoseStamped.pose.position.x+0.05, motorGoalPoseStamped.pose.position.y, -motorGoalPoseStamped.pose.position.z;
            // std::cout << "Goal Position: " << xg << std::endl;
            

            //theta = Get joint state
            if (group->getNextFeedback(group_feedback))
            {
                thetas = group_feedback.getPosition(); 
            }
            
            //[x,y,z of the end effector] -- x0
            Eigen::Matrix4d transform;
            //model->getFK(hebi::robot_model::FrameType::CenterOfMass, thetas, transforms)
            model->getEndEffector(thetas,transform);

            //convert 3x3 rotation transform roll0, pitch0, yaw0
            double roll0 = atan2(transform(2,1), transform(2,2));
            double pitch0 = atan2(-transform(2,0), sqrt(pow(transform(2,1),2)+pow(transform(2,2),2)));
            double yaw0 = atan2(transform(1,0), transform(0,0));
            // tf::Quaternion tempTFQuat;
            // tempTFQuat.setRPY(roll0, pitch0, yaw0);
            // tempTFQuat.normalize();
            // geometry_msgs::Quaternion tempGeoQuat;
            // tempGeoQuat = tf::toMsg(tempTFQuat);


            // Feng Xiang
            // Eigen::VectorXd x0(6);
            // x0 << transform(0,3), transform(1,3), transform(2,3), roll0, pitch0, yaw0;

            Eigen::Vector3d x0;
            x0 << transform(0,3), transform(1,3), transform(2,3);

            // std::cout << "Goal:" << xg << std::endl;
            std::cout << "Current: " << x0 << std::endl;

            //Compute Jacobian -- J
            //[2d matrix of joint angles ]
            // hebi::robot_model::MatrixXdVector J;
            Eigen::MatrixXd J;
            // model->getJ(hebi::robot_model::FrameType::CenterOfMass, thetas, J);
            model->getJEndEffector(thetas, J);
            // std::cout << "[RESOLVED RATE] Loop - acquired Jacobian from HRDF model." << std::endl;
            // Feng Xiang
            // Eigen::MatrixXd ee_J = J[J.size()-1].block(0,0,3,4);
            Eigen::MatrixXd ee_J = J.block(0,0,3,4);
            

            // Feng Xiang
            // Eigen::VectorXd err(6);
            Eigen::Vector3d err;
            err = xg - x0;
            // if(err.norm() < 0.1) dt = 0.2;
            // else dt = 0.5;

            if (W.isIdentity(0.1))
            {
                thetadot = ee_J.transpose()*(ee_J*ee_J.transpose()).inverse()*err;
            }
            else
            {
                thetadot = W.inverse()*ee_J.transpose()*(ee_J*W.inverse()*ee_J.transpose()).inverse()*err;
            }

            thetas += dt*thetadot;

            // ROS_INFO(thetas);
            // std::cout << thetas << std::endl;
            
            //command_angles(theta)
            groupCommand.setPosition(thetas);
            // std::cout << "[RESOLVED RATE] Loop - Sending updated theta positions to motors." << std::endl;
            group->sendCommand(groupCommand);

        }

        ros::spinOnce();
        rate.sleep();
    }

    std::cout << "Code is Done." << std::endl;
}

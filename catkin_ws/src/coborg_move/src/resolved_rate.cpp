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

#include "std_msgs/Bool.h"
#include <gb_visual_detection_3d_msgs/goal_msg.h>

tf::StampedTransform* prevTransform;
tf::TransformListener* globalListener;
gb_visual_detection_3d_msgs::goal_msg goal; 
bool enable_rr;

geometry_msgs::Pose poseMotionDetection(const geometry_msgs::Pose& pose_msg)
{    
    tf::StampedTransform transform;
    geometry_msgs::Pose target_pose;

    try
    {
        // TODO: acquire the transform once
        // FORNOW: update transform parameters at every callback interval
        //listener.waitForTransform("/world", "/camera_link", ros::Time(0), ros::Duration(3.0));
        globalListener->waitForTransform("/t265_odom", "/motor1_link/INPUT_INTERFACE",ros::Time::now(), ros::Duration(3.0));
        globalListener->lookupTransform("/t265_odom", "/motor1_link/INPUT_INTERFACE",ros::Time::now(), transform);

        // FORNOW: only goal position is updated b/c 3DoF robot arm cannot solve 6DoF goal every time
        target_pose.position.x = -(transform.getOrigin().getX()-prevTransform->getOrigin().getX()) + pose_msg.position.x;
        target_pose.position.y = -(transform.getOrigin().getY()-prevTransform->getOrigin().getY()) + pose_msg.position.y;
        target_pose.position.z = -(transform.getOrigin().getZ()-prevTransform->getOrigin().getZ()) + pose_msg.position.z;

        ROS_INFO("Transforms are: x: %f, y: %f: z: %f", target_pose.position.x,target_pose.position.y, target_pose.position.z);

        // robot arm can now move to updated goal pose
        return target_pose;
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return pose_msg;
    }
}

void goal_callback(const gb_visual_detection_3d_msgs::goal_msg::ConstPtr& goal_msg)
{
    tf::StampedTransform transform;

    bool goal_got = false;
    while(!goal_got)
    {
        try
        {
            // TODO: acquire the transform once
            // FORNOW: update transform parameters at every callback interval
            //listener.waitForTransform("/world", "/camera_link", ros::Time(0), ros::Duration(3.0));
            globalListener->waitForTransform("/d400_link", "/t265_odom", ros::Time::now(), ros::Duration(3.0));
            globalListener->lookupTransform("/d400_link", "/t265_odom", ros::Time::now(), transform);

            // FORNOW: only goal position is updated b/c 3DoF robot arm cannot solve 6DoF goal every time
            goal.x = -transform.getOrigin().getX() + goal_msg->x;
            goal.y = -transform.getOrigin().getY() + goal_msg->y;
            goal.z = -transform.getOrigin().getZ() + goal_msg->z;

            ROS_INFO("Transforms are: x: %f, y: %f: z: %f", goal.x,goal.y, goal.z);

            // robot arm can now move to updated goal pose
            goal_got = true;
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            goal_got = false;
        }
    }
}

void enable_rr_callback(const std_msgs::Bool::ConstPtr& bool_msg)
{
    enable_rr = bool_msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "resolved_rate_planner");
    ros::NodeHandle node;

    // Establish initial values
    tf::TransformListener listener;
    globalListener = &listener;

    tf::StampedTransform tempTrans;
    globalListener->waitForTransform("/t265_odom", "/end_link/INPUT_INTERFACE", ros::Time::now(), ros::Duration(3.0));
    globalListener->lookupTransform("/t265_odom", "/end_link/INPUT_INTERFACE", ros::Time::now(), tempTrans);
    prevTransform = &tempTrans;

    enable_rr = false;

    // Get goal position once it's published by vision nodes (same goal move_it picks up)
    ros::Subscriber goal_pos_sub = node.subscribe("/goal",1,goal_callback);
    // Topic to trigger resolved rate to begin
    ros::Subscriber tigger_rr_sub = node.subscribe("/enable_rr",1,enable_rr_callback);


    // inititalize HEBI API
    std::vector<std::string> families;
    families = {"X5-9","X5-9","X5-4"};
    std::vector<std::string> names;
    names = {"base_1", "elbow_2", "wrist_3"};

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

    hebi::GroupCommand groupCommand(group->size());

    hebi::GroupFeedback group_feedback(group->size());
    Eigen::VectorXd thetas(group->size());
    Eigen::VectorXd thetadot(group->size());

    float dt = 0.01;
    Eigen::MatrixXd W(group->size(),group->size());
    W.setIdentity();
    std::unique_ptr<hebi::robot_model::RobotModel> model = hebi::robot_model::RobotModel::loadHRDF("coborgarm-temp.hrdf");

    ros::Rate rate(20.0);
    while(ros::ok())
    {
        if(enable_rr)
        {
            //xg = goal
            //convert gb_visual_detection_3d_msgs::goal_msg to geometry_msgs::Pose
            geometry_msgs::Pose goal_converted;
            goal_converted.position.x = goal.x;
            goal_converted.position.y = goal.y;
            goal_converted.position.z = goal.z;
            //update geometry_msgs::Pose to most recent /t265_odom transform
            geometry_msgs::Pose target_pose = poseMotionDetection(goal_converted);
            //convert geometry_msgs::Pose to Eigen::Vector3d
            Eigen::Vector3d xg;
            xg << target_pose.position.x, target_pose.position.y, target_pose.position.z;

            //theta = Get joint state
            if (group->getNextFeedback(group_feedback))
            {
                thetas = group_feedback.getPosition(); 
            }
            
            //[x,y,z of the end effector] -- x0
            Eigen::Matrix4d transform;
            //model->getFK(hebi::robot_model::FrameType::CenterOfMass, thetas, transforms)
            model->getEndEffector(thetas,transform);
            Eigen::Vector3d x0;
            x0 << transform(0,3), transform(1,3), transform(2,3);

            //Compute Jacobian -- J
            //[2d matrix of joint angles ]
            hebi::robot_model::MatrixXdVector J;
            model->getJ(hebi::robot_model::FrameType::CenterOfMass, thetas, J);
            Eigen::MatrixXd ee_J = J[J.size()-1].block(0,0,3,4);


            if (W.isIdentity(0.1))
            {
                thetadot = ee_J.transpose()*(ee_J*ee_J.transpose()).inverse()*(xg - x0);
            }
            else
            {
                thetadot = W.inverse()*ee_J.transpose()*(ee_J*W.inverse()*ee_J.transpose()).inverse()*(xg - x0);
            }

            // if (W.isIdentity(0.1))
            // {   
            //     for (int it = 0; it < thetadot.size(); it++)
            //     {
            //         Eigen::MatrixXd _temp_J = J[it].transpose()*(J[it]*J[it].transpose()).inverse(); // (3x4)
            //         Eigen::VectorXd _temp_diff = xg - x0; // (3x1)
            //         thetadot[it] = _temp_J*_temp_diff;
            //     }
            // }
            // else
            // {
            //     for (int it = 0; it < thetadot.size(); it++)
            //     {
            //         thetadot[it] = W.inverse()*J[it].transpose()*(J[it]*W.inverse()*J[it].transpose()).inverse()*(xg-x0);
            //     }
            // }

            //thetadot = inv(W)*J.T*inv(J*inv(W)*J.T)*(xg-x0)
            //if (W.isIdentity(0.1)) {thetadot = J.T*inv(J*J.T)*(xg-x0)}

            //thetas = thetas + dt*thetadot
            thetas += dt*thetadot;
            
            //command_angles(theta)
            groupCommand.setPosition(thetas);
            group->sendCommand(groupCommand);

        }
        ros::spinOnce();
        rate.sleep();
    }
}
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <gb_visual_detection_3d_msgs/goal_msg.h>
#include <vector>
#include <cmath>
#include <math.h>
#include <boost/scoped_ptr.hpp>
#include <tf/transform_listener.h>
#include <tf/tf.h>


#include "Eigen/Eigen"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PointStamped.h"

#define HZ 5
double tf_time_offset = 0.1;
double cam1_prevTime = 0.0;
double cam2_prevTime = 0.0;
ros::Time prevTime;

class GoalGetter
{
public:
    GoalGetter()
    {
        // subscribe to state_output rostopic
        state_output_sub_ = nh_.subscribe("state_output", 1, &GoalGetter::state_output_callback, this);

        // publish to state_input rostopic
        state_input_pub_ = nh_.advertise<std_msgs::Int32>("state_input", 1);
        

        goal_pub_ = nh_.advertise<geometry_msgs::PointStamped>("goal", 1);

        // TODO: publishers for manipulation node

        curr_state_ = 0;  // main state
        goalSetPose << 0.0,0.0,0.0;
        measuredNormal << 0.0,0.0,0.0;
    }

    void step(){
        // process the goals
        if (curr_state_ == 2 && !goal_received){
            processGoal(goalSetPose, measuredNormal);  // goalSetPose: relative to /t265_odom_frame
            goal_received = true;
        }

        std::cout << "Current state is: " << curr_state_ << std::endl;
        std::cout << "Goal is: " <<  goal_point << std::endl;    

        if (curr_state_ ==2 || curr_state_==3){
            if (goalSetPose(0)!=0.0){
                // keep publishing the point (relative to the world)
                convertGoalPoint(goal_point, goalSetPose);
            
                // TODO: surface normal
            }
        }

    }

private:
    void state_output_callback(const std_msgs::Int32::ConstPtr& outputVal)
    {
        curr_state_ = outputVal->data;

        if (outputVal->data == 2)
        {
            // start vision goal processing
            commandReceived_.data = 1;
            state_input_pub_.publish(commandReceived_);
        }
    }


    Eigen::MatrixXf generalPositionUpdate(const gb_visual_detection_3d_msgs::goal_msg::ConstPtr& goal_msg, double &prevTime)
    {
        if (ros::Time::now().toSec() - prevTime > tf_time_offset)
        {   
            Eigen::MatrixXf transformedGoalNormal(2,3);  // first row is goal, second is surface normal
            geometry_msgs::PointStamped goal_point;
            goal_point.header.frame_id = goal_msg -> header.frame_id;
            goal_point.header.stamp = goal_msg -> header.stamp;
            prevTime = goal_msg -> header.stamp.toSec();
            goal_point.point.x = goal_msg -> x;
            goal_point.point.y = goal_msg -> y;
            goal_point.point.z = goal_msg -> z;

            // geometry_msgs::PointStamped goal_normal;
            // goal_normal.header.frame_id = goal_msg -> header.frame_id;
            // goal_normal.header.stamp = goal_msg -> header.stamp;
            // goal_normal.point.x = goal_msg -> normal_x;
            // goal_normal.point.y = goal_msg -> normal_y;
            // goal_normal.point.z = goal_msg -> normal_z;


            try{
                tfListener_.transformPoint("/t265_odom_frame", goal_point, goal_point);
                transformedGoalNormal(0,0) = goal_point.point.x;
                transformedGoalNormal(0,1) = goal_point.point.y;
                transformedGoalNormal(0,2) = goal_point.point.z;

                // globalListener->transformPoint("/world", goal_normal, goal_normal);
                // transformedGoalNormal(1,0) = goal_normal.point.x;
                // transformedGoalNormal(1,1) = goal_normal.point.y;
                // transformedGoalNormal(1,2) = goal_normal.point.z;

                prevTime = ros::Time::now().toSec();
                return transformedGoalNormal; 

            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s", ex.what());
                Eigen::MatrixXf nullVect;
                return nullVect;

            }

        }

    }

    void processGoal(Eigen::Vector3d& goalSetPose, Eigen::Vector3d& measuredNormal){
        // convert goal from camera frame to /t265_odom_frame and take care of edge cases

        ros::Duration(1.0).sleep();
        boost::shared_ptr<gb_visual_detection_3d_msgs::goal_msg const> goalpose_cam1;
        boost::shared_ptr<gb_visual_detection_3d_msgs::goal_msg const> goalpose_cam2;

        while(true){
            try
            {
                goalpose_cam1 = ros::topic::waitForMessage<gb_visual_detection_3d_msgs::goal_msg>("/goal_cam1", ros::Duration(1.0));
            }
            catch(std::exception e)
            {
                std::cout << "Cam1 not found" << std::endl;
            }
            
            try
            {
                goalpose_cam2 = ros::topic::waitForMessage<gb_visual_detection_3d_msgs::goal_msg>("/goal_cam2", ros::Duration(1.0));
            }
            catch(std::exception e)
            {
                std::cout << "Cam2 not found" << std::endl;
            }
            if (goalpose_cam1 || goalpose_cam2)
                break;
        }
        
        
        goalSetPose(0) = 0;
        goalSetPose(1) = 0;
        goalSetPose(2) = 0;
        int totalCameras = 0;

        if (goalpose_cam1 != NULL)
        {
            ROS_INFO("Cam 01 Position Received.");
            // tf transform to cam1_link -> world frame
            Eigen::MatrixXf cam1_goalSetPose = generalPositionUpdate(goalpose_cam1, cam1_prevTime);
            
            
            if (cam1_goalSetPose(0,0) != 0 && cam1_goalSetPose(0,0)<=1.5)
            {
                double transformed_x = cam1_goalSetPose(0,0); // placeholder
                double transformed_y = cam1_goalSetPose(0,1); // placeholder
                double transformed_z = cam1_goalSetPose(0,2); // placeholder

                // add to goal pose
                goalSetPose(0) += transformed_x;
                goalSetPose(1) += transformed_y;
                goalSetPose(2) += transformed_z;
                totalCameras++;
            }

        }

        if (goalpose_cam2 != NULL)
        {
            ROS_INFO("Cam 02 Position Received.");
            // tf transform to cam2_link frame -> world frame
            Eigen::MatrixXf cam2_goalSetPose = generalPositionUpdate(goalpose_cam2, cam2_prevTime);
            if (cam2_goalSetPose(0,0) != 0 && cam2_goalSetPose(0,0)<=1.5)
            {
                double transformed_x = cam2_goalSetPose(0,0); // placeholder
                double transformed_y = cam2_goalSetPose(0,1); // placeholder
                double transformed_z = cam2_goalSetPose(0,2); // placeholder

                // add to goal pose
                goalSetPose(0) += transformed_x;
                goalSetPose(1) += transformed_y;
                goalSetPose(2) += transformed_z;
                totalCameras++;
            }

        }

        goalSetPose(0) /= totalCameras;
        goalSetPose(1) /= totalCameras;
        goalSetPose(2) /= totalCameras;

        ROS_INFO_STREAM("Goal Position Reading: \n" << goalSetPose << "\n");

        measuredNormal << 0.0,0.0,0.0;
        if (goalpose_cam1!=NULL){
            // transform to normal frame

            measuredNormal[0] += goalpose_cam1->normal_x;
            measuredNormal[1] += goalpose_cam1->normal_y;
            measuredNormal[2] += goalpose_cam1->normal_z;
        }

        if (goalpose_cam2!= NULL){
            measuredNormal[0] += goalpose_cam2->normal_x;
            measuredNormal[1] += goalpose_cam2->normal_y;
            measuredNormal[2] += goalpose_cam2->normal_z;
        }
    }

    void convertGoalPoint(geometry_msgs::PointStamped& goal_point, Eigen::Vector3d& goalSetPose){
        // convert goal position from /t265_odom_frame to /world

        if (ros::Time::now().toSec() - prevTime.toSec() > tf_time_offset)
        {   
            // transform it to point stamp
            // goal_point.header.stamp = ros::Time::now();
            goal_point.header.frame_id = "/t265_odom_frame";
            goal_point.header.stamp = prevTime;
            goal_point.point.x = goalSetPose(0);
            goal_point.point.y = goalSetPose(1);
            goal_point.point.z = goalSetPose(2);
            std::cout << "Time is  "<< prevTime << std::endl;
            // transform it to /world
            try{
                tfListener_.transformPoint("/world", goal_point, goal_point);
                goal_pub_.publish(goal_point);
                prevTime = ros::Time::now();
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s", ex.what());
            }
        }
    }

    ros::NodeHandle nh_;
    ros::Publisher state_input_pub_;
    ros::Publisher goal_pub_;
    ros::Subscriber state_output_sub_;
    tf::TransformListener tfListener_;
    int curr_state_;
    std_msgs::Int32 commandReceived_;
    Eigen::Vector3d goalSetPose;
    Eigen::Vector3d measuredNormal;
    bool goal_received = false;
    geometry_msgs::PointStamped goal_point;
};



int main(int argc, char ** argv)
{
  ros::init(argc, argv, "goal_getter");

  GoalGetter goal_getter;

  // Configure the loop frequency (Hertzios):

  ros::Rate loop_rate(HZ);

  while (ros::ok())
  {
    ros::spinOnce();
    goal_getter.step();
    loop_rate.sleep();
  }

  return 0;
}
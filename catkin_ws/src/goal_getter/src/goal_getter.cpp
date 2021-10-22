#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <gb_visual_detection_3d_msgs/goal_msg.h>
#include <goal_getter/GoalPose.h>
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
#include "geometry_msgs/PoseStamped.h"

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
        

        goal_pub_ = nh_.advertise<goal_getter::GoalPose>("goal", 1);

        curr_state_ = 0;  // main state
        goalSetPoint << 0.0,0.0,0.0;
    }

    void step(){
        // process the goals
        if (curr_state_ == 2 && !goal_received){
            processGoal(goalSetPoint, goal_normal);  // goalSetPoint: relative to /t265_odom_frame
            goal_received = true;
        }

        std::cout << "Current state is: " << curr_state_ << std::endl;
        std::cout << "Goal is: " <<  goal_point << std::endl;    

        if (curr_state_ ==2 || curr_state_==3){
            if (goalSetPoint(0)!=0.0){
                // convert to /world frame and keep publishing the point
                convertToWorld(goal_point, goal_normal, goalSetPoint);
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

    void updateNormal(geometry_msgs::PoseStamped& goal_normal_pose, Eigen::Vector3f& goal_normal_vec){
        Eigen::Vector3f goal_normal_x_axis = goal_normal_vec;
        // Set the z axis temporarily to a random variable
        Eigen::Vector3f goal_normal_z_axis;
        goal_normal_z_axis << 1, 1, 1;
        // Set the y axis to be perpendicular to the x axis and the temporary z axis
        Eigen::Vector3f goal_normal_y_axis = goal_normal_x_axis.cross(goal_normal_z_axis.normalized());
        // If the x and temporary z axes happened to be the same, the y axis will be 0, so retry with a new temporary vector
        Eigen::Vector3f zero;
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
        Eigen::Matrix3f goal_normal_rotation_matrix;
        goal_normal_rotation_matrix << goal_normal_x_axis.normalized(), goal_normal_y_axis.normalized(), goal_normal_z_axis.normalized();
        // Convert the rotation matrix to a quaternion
        Eigen::Quaternionf goal_normal_quaternion(goal_normal_rotation_matrix);
        // Fill geometry pose message with goal orientation information
        goal_normal_pose.pose.orientation.x = goal_normal_quaternion.x();
        goal_normal_pose.pose.orientation.y = goal_normal_quaternion.y();
        goal_normal_pose.pose.orientation.z = goal_normal_quaternion.z();
        goal_normal_pose.pose.orientation.w = goal_normal_quaternion.w();
    }

    void convertToOdom(geometry_msgs::PointStamped& goal_point, geometry_msgs::PoseStamped& goal_normal, const gb_visual_detection_3d_msgs::goal_msg::ConstPtr& goal_msg, double &prevTime)
    {
        if (ros::Time::now().toSec() - prevTime > tf_time_offset)
        {   
            // Update point stamp
            goal_point.header.frame_id = goal_msg -> header.frame_id;
            goal_point.header.stamp = goal_msg -> header.stamp;
            goal_point.point.x = goal_msg -> x;
            goal_point.point.y = goal_msg -> y;
            goal_point.point.z = goal_msg -> z;

            prevTime = goal_msg -> header.stamp.toSec();

            // update surface normal pose stamp
            goal_normal.header.frame_id = goal_msg -> header.frame_id;
            goal_normal.header.stamp = goal_msg -> header.stamp;
            goal_normal.pose.position.x = goal_msg -> x;
            goal_normal.pose.position.y = goal_msg -> y;
            goal_normal.pose.position.z = goal_msg -> z;

            Eigen::Vector3f goal_normal_vec;
            goal_normal_vec(0) = goal_msg -> normal_x;
            goal_normal_vec(1) = goal_msg -> normal_y;
            goal_normal_vec(2) = goal_msg -> normal_z;

            updateNormal(goal_normal, goal_normal_vec);

            try{
                tfListener_.transformPoint("/t265_odom_frame", goal_point, goal_point);
                tfListener_.transformPose("/t265_odom_frame", goal_normal, goal_normal);

                prevTime = ros::Time::now().toSec();
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s", ex.what());
            }
        }
    }

    void processGoal(Eigen::Vector3d& goalSetPoint, geometry_msgs::PoseStamped& goal_normal){
        // convert goal from /camera frame to /t265_odom_frame and take care of edge cases

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
        
        goalSetPoint(0) = 0;
        goalSetPoint(1) = 0;
        goalSetPoint(2) = 0;
        int totalCameras = 0;

        geometry_msgs::PointStamped goal_point_cam1;
        geometry_msgs::PoseStamped goal_normal_cam1;
        geometry_msgs::PointStamped goal_point_cam2;
        geometry_msgs::PoseStamped goal_normal_cam2;

        if (goalpose_cam1 != NULL)
        {
            ROS_INFO("Cam 01 Position Received.");
            // tf transform to cam1_link -> odom frame
            convertToOdom(goal_point_cam1, goal_normal_cam1, goalpose_cam1, cam1_prevTime);
            
            if (goal_point_cam1.point.x != 0 && goal_point_cam1.point.x<=1.5)
            {
                double transformed_x = goal_point_cam1.point.x; // placeholder
                double transformed_y = goal_point_cam1.point.y; // placeholder
                double transformed_z = goal_point_cam1.point.z; // placeholder

                // add to goal pose
                goalSetPoint(0) += transformed_x;
                goalSetPoint(1) += transformed_y;
                goalSetPoint(2) += transformed_z;
                totalCameras++;
            }
        }

        if (goalpose_cam2 != NULL)
        {
            ROS_INFO("Cam 02 Position Received.");
            // tf transform to cam2_link frame -> odom frame
            convertToOdom(goal_point_cam2, goal_normal_cam2, goalpose_cam2, cam2_prevTime);

            if (goal_point_cam2.point.x != 0 && goal_point_cam2.point.x<=1.5)
            {
                double transformed_x = goal_point_cam2.point.x; // placeholder
                double transformed_y = goal_point_cam2.point.y; // placeholder
                double transformed_z = goal_point_cam2.point.z; // placeholder

                // add to goal pose
                goalSetPoint(0) += transformed_x;
                goalSetPoint(1) += transformed_y;
                goalSetPoint(2) += transformed_z;
                totalCameras++;
            }
        }

        goalSetPoint(0) /= totalCameras;
        goalSetPoint(1) /= totalCameras;
        goalSetPoint(2) /= totalCameras;

        ROS_INFO_STREAM("Goal Position Reading: \n" << goalSetPoint << "\n");

        if (goalpose_cam1!=NULL){
            goal_normal_cam1.pose.position.x = goalSetPoint(0);
            goal_normal_cam1.pose.position.y = goalSetPoint(1);
            goal_normal_cam1.pose.position.z = goalSetPoint(2);
            goal_normal = goal_normal_cam1;
        }

        if (goalpose_cam2!= NULL){
            goal_normal_cam2.pose.position.x = goalSetPoint(0);
            goal_normal_cam2.pose.position.y = goalSetPoint(1);
            goal_normal_cam2.pose.position.z = goalSetPoint(2);
            goal_normal = goal_normal_cam2;
        }
    }

    void convertToWorld(geometry_msgs::PointStamped& goal_point, geometry_msgs::PoseStamped& goal_normal, Eigen::Vector3d& goalSetPoint){
        // convert goal position/surface normal from /t265_odom_frame to /world

        if (ros::Time::now().toSec() - prevTime.toSec() > tf_time_offset)
        {   
            // transform it to point stamp
            goal_point.header.frame_id = "/t265_odom_frame";
            goal_point.header.stamp = prevTime;
            goal_point.point.x = goalSetPoint(0);
            goal_point.point.y = goalSetPoint(1);
            goal_point.point.z = goalSetPoint(2);
            std::cout << "Time is  "<< prevTime << std::endl;
            // transform it to /world
            try{
                tfListener_.transformPoint("/world", goal_point, goal_point);
                tfListener_.transformPose("/world", goal_normal, goal_normal);

                goal_getter::GoalPose goal_msg;
                goal_msg.goal_point = goal_point;
                goal_msg.goal_normal = goal_normal;
                goal_pub_.publish(goal_msg);
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
    Eigen::Vector3d goalSetPoint;
    bool goal_received = false;
    geometry_msgs::PoseStamped goal_normal;
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
#include <ros/ros.h>
#include <gb_visual_detection_3d_msgs/BoundingBox3d.h>
#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <gb_visual_detection_3d_msgs/goal_msg.h>
#include <vector>
#include <cmath>
#include <math.h>
#include <boost/scoped_ptr.hpp>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>


#include "Eigen/Eigen"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseStamped.h"

#define HZ 5
double tf_time_offset = 0.1;
double cam1_prevTime = 0.0;
double cam2_prevTime = 0.0;

class GoalGetter
{
public:
    GoalGetter()
    {
        // subscribe to state_output rostopic
        state_output_sub_ = nh_.subscribe("state_output", 1, state_output_callback);

        // publish to state_input rostopic
        state_input_pub_ = nh_.advertise<std_msgs::Int32>("state_input", 1);
        state_input_ptr_ = &state_input_pub_;

        curr_state_ = 0;  // main state
        goalSetPose(0,0,0);
        measuredNormal(0,0,0);
    }

    void step(){
        // process the goals
        if (curr_state_ == 2){
            processGoal(goalSetPose, measuredNormal);
            // publishGoal(goalSetPose, measuredNormal);
        }

    }

private:
    void state_output_callback(const std_msgs::Int32::ConstPtr& outputVal)
    {

        if (outputVal->data == 2)
        {
            // start vision goal processing
            commandReceived_.data = 1;
            state_input_ptr->publish(commandReceived_);
            curr_state_ = 2;

        }
    }


    Eigen::Vector3d generalPositionUpdate(const gb_visual_detection_3d_msgs::goal_msg::ConstPtr& goal_msg, double &prevTime)
    {
        if (ros::Time::now().toSec() - prevTime > tf_time_offset)
        {
            tf::StampedTransform transform;
            ros::Time currTime = ros::Time::now();
            Eigen::Vector3d tempVect;

            std::string source_frame = goal_msg->header.frame_id;
            geometry_msgs::PoseStamped tempPoseStamped;
            tempPoseStamped.header.frame_id = goal_msg->header.frame_id;
            tempPoseStamped.header.stamp = goal_msg->header.stamp;
            tempPoseStamped.pose.position.x = goal_msg->x;
            tempPoseStamped.pose.position.y = goal_msg->y;
            tempPoseStamped.pose.position.z = goal_msg->z;



            double camRoll = atan(goal_msg->normal_z / goal_msg->normal_y);
            double camPitch = 3.14159 - atan(goal_msg->normal_z / goal_msg->normal_x);
            double camYaw = atan(goal_msg->normal_y / goal_msg->normal_x);

            // Eigen::Vector3d cameraXAxis(1,0,0);
            // Eigen::Vector3d cameraYAxis(0,1,0);

            // Eigen::Vector3d normVectX(goal_msg->normal_x, goal_msg->normal_y, goal_msg->normal_z);
            tf::Quaternion tempQuaternion;
            // this is wrong (normal x is not a radian)
            tempQuaternion.setRPY(camRoll, camPitch, camYaw);
            tempPoseStamped.pose.orientation.x = tempQuaternion.x;
            tf::quaternionTFToMsg(tempQuaternion, tempPoseStamped.pose.orientation);
            


            try{
                // globalListener->waitForTransform("/world", source_frame.c_str(), currTime, ros::Duration(3.0));
                // globalListener->lookupTransform("/world", source_frame.c_str(), currTime, transform);
                // geometry_msgs::TransformStamped frame_leap_motion = globalListener->lookupTransform("/world", source_frame.c_str(), currTime, ros::Duration(1.0));
                // presetPositionUpdate(transform, homePose);
                globalListener->transformPose("/world", tempPoseStamped, tempPoseStamped);
                tempVect(0) = tempPoseStamped.pose.position.x;
                tempVect(1) = tempPoseStamped.pose.position.y;
                tempVect(2) = tempPoseStamped.pose.position.z;
                // tempVect(0) = -transform.getOrigin().getX() + goal_msg->x;
                // tempVect(1) = -transform.getOrigin().getY() + goal_msg->y;
                // tempVect(2) = -transform.getOrigin().getZ() + goal_msg->z;
                
                // ROS_INFO("Transform rotation is: x: %f, y: %f, z: %f, w: %f",transform.getRotation().x(),transform.getRotation().y(),transform.getRotation().z(),transform.getRotation().w());
                // ROS_INFO("Transform translation is: x: %f, y: %f: z: %f", transform.getOrigin().getX(),transform.getOrigin().getY(), transform.getOrigin().getZ());

                prevTime = ros::Time::now().toSec();
                return tempVect; 

            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s", ex.what());
                Eigen::Vector3d nullVect;
                return nullVect;

            }

        }

    }

    void processGoal(Eigen::Vector3d& goalSetPose, Eigen::Vector3d& measuredNormal){
        // goal is relative to camera frame

        ros::Duration(1.0).sleep();
        boost::shared_ptr<gb_visual_detection_3d_msgs::goal_msg const> goalpose_cam1;
        boost::shared_ptr<gb_visual_detection_3d_msgs::goal_msg const> goalpose_cam2;
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
        
        
        goalSetPose(0) = 0;
        goalSetPose(1) = 0;
        goalSetPose(2) = 0;
        int totalCameras = 0;

        if (goalpose_cam1 != NULL)
        {
            ROS_INFO("Cam 01 Position Received.");
            // tf transform to cam1_link -> world frame
            Eigen::Vector3d cam1_goalSetPose = generalPositionUpdate(goalpose_cam1, cam1_prevTime);
            
            
            if (cam1_goalSetPose(0) != 0)
            {
                double transformed_x = cam1_goalSetPose(0); // placeholder
                double transformed_y = cam1_goalSetPose(1); // placeholder
                double transformed_z = cam1_goalSetPose(2); // placeholder

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
            Eigen::Vector3d cam2_goalSetPose = generalPositionUpdate(goalpose_cam2, cam2_prevTime);
            if (cam2_goalSetPose(0) != 0)
            {
                double transformed_x = cam2_goalSetPose(0); // placeholder
                double transformed_y = cam2_goalSetPose(1); // placeholder
                double transformed_z = cam2_goalSetPose(2); // placeholder

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

        measuredNormal(0,0,0);
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



    ros::NodeHandle nh_;
    ros::Publisher* state_input_ptr_;
    ros::Subscriber state_output_sub_;
    int curr_state_;
    std_msgs::Int32 commandReceived_;
    Eigen::Vector3d goalSetPose;
    Eigen::Vector3d measuredNormal;
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
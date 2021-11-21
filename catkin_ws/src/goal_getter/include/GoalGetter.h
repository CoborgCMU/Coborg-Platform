#ifndef GOAlGETTER_H
#define GOALGETTER_H

#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <gb_visual_detection_3d_msgs/goal_msg.h>
#include <goal_getter/GoalPose.h>
#include <vector>
#include <queue>
#include <boost/scoped_ptr.hpp>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>


#include "Eigen/Eigen"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseStamped.h"


class GoalGetter
{
  public:
    GoalGetter();

    void step();

  private:
    void state_output_callback(const std_msgs::Int32::ConstPtr& outputVal);
    void updateNormal(geometry_msgs::PoseStamped& goal_normal_pose, Eigen::Vector3f& goal_normal_vec);
    void convertToOdom(geometry_msgs::PoseStamped& goal_normal, const gb_visual_detection_3d_msgs::goal_msg::ConstPtr& goal_msg, double& prevTimeLocal);
    void processGoal(geometry_msgs::PoseStamped& goalNormalSetPose);
    void convertToWorld(geometry_msgs::PoseStamped& goal_normal, geometry_msgs::PoseStamped& goalNormalSetPose);


    ros::NodeHandle nh_;
    ros::Publisher state_input_pub_;
    ros::Publisher goal_pub_;
    ros::Subscriber state_output_sub_;
    tf::TransformListener tfListener_;
    tf::TransformBroadcaster tfBroadcaster_;
    int curr_state_;
    std_msgs::Int32 visionFeedback_;
    Eigen::Vector3d goalSetPoint; // average goal point relative to t265_odom_frame
    geometry_msgs::PoseStamped goalNormalSetPose;
    bool goal_received;
    bool goal_normal_computed;
    geometry_msgs::PoseStamped goal_normal;
    geometry_msgs::PoseStamped goal_normal_motor1;
    geometry_msgs::PoseStamped goal_normal_sum;
    geometry_msgs::PoseStamped goal_normal_motor1_sum;
    std::queue<geometry_msgs::PoseStamped> goal_normal_queue;
    std::queue<geometry_msgs::PoseStamped> goal_normal_motor1_queue;
    int window_size = 1;

     // timing variables for ROS
    double tf_time_offset = 0.15;
    double cam1_prevTime = 0.0;
    double cam2_prevTime = 0.0;
    ros::Time prevTime;

};

#endif  // GOALGETTER_H

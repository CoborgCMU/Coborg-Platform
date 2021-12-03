#include <ros/ros.h>

#include "GoalGetter.h"


#define HZ 20

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "goal_getter");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  GoalGetter goal_getter;

  // Configure the loop frequency (Hertzios):

  ros::Rate loop_rate(HZ);

  while (ros::ok())
  {
    goal_getter.step();
  }

  return 0;
}

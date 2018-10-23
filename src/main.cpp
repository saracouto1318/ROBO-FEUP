# include "../include/reactive_robot.h"

int main(int argc,char **argv)
{
  ros::init(argc, argv, "reactive_robot", ros::init_options::AnonymousName);
  reactive_robot::WallFollowing obj(argc, argv);
  ros::spin();
  return 0;
}

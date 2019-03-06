#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "tango_msgs/motor_commands.h"

std_msgs::Float32 lmotorcmd_msg;
std_msgs::Float32 rmotorcmd_msg;
tango_msgs::motor_commands mmsg;

void lmotorCmdCallback(const std_msgs::Float32& lmotorcmd_msg)
{
    mmsg.l_motor = lmotorcmd_msg.data;
}

void rmotorCmdCallback(const std_msgs::Float32& rmotorcmd_msg)
{
    mmsg.r_motor = rmotorcmd_msg.data;
}
	
int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_controls");
  ros::NodeHandle n;
  ros::Rate loop_rate(20);   
  ros::Publisher mc_pub = n.advertise<tango_msgs::motor_commands>("motor_commands", 1000);
  ros::Subscriber lmotor_sub = n.subscribe("lmotor_cmd", 1000, lmotorCmdCallback);
  ros::Subscriber rmotor_sub = n.subscribe("rmotor_cmd", 1000, rmotorCmdCallback);
  int count = 0;
  while (ros::ok())
  {
    mc_pub.publish(mmsg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  ros::spin();
  return 0;
}

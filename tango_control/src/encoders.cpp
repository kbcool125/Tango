#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "tango_msgs/wheel_encoders.h"

std_msgs::Int16 lwheelenc_msg;
std_msgs::Int16 rwheelenc_msg;
tango_msgs::wheel_encoders wmsg;

void wheelEncodersCallback(const tango_msgs::wheel_encoders& wmsg)
{
    lwheelenc_msg.data = wmsg.l_count;
    rwheelenc_msg.data = wmsg.r_count;
}
	
int main(int argc, char **argv)
{
  ros::init(argc, argv, "wheel_encoders");
  ros::NodeHandle n;
  ros::Rate loop_rate(20);   
  ros::Publisher lwheel_pub = n.advertise<std_msgs::Int16>("lwheel", 1000);
  ros::Publisher rwheel_pub = n.advertise<std_msgs::Int16>("rwheel", 1000);
  ros::Subscriber wc_sub = n.subscribe("wheel_encoders", 1000, wheelEncodersCallback);
  int count = 0;
  while (ros::ok())
  {
    lwheel_pub.publish(lwheelenc_msg);
    rwheel_pub.publish(rwheelenc_msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  ros::spin();
  return 0;
}

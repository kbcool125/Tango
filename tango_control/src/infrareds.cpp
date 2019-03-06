#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "tango_msgs/infrareds.h"

sensor_msgs::Range front_c_ir_msg;
sensor_msgs::Range front_r_inner_ir_msg;
sensor_msgs::Range front_r_outer_ir_msg;
sensor_msgs::Range rear_r_ir_msg;
sensor_msgs::Range rear_l_ir_msg;
sensor_msgs::Range front_l_outer_ir_msg;
sensor_msgs::Range front_l_inner_ir_msg;
tango_msgs::infrareds irmsg;

void infraredsCallback(const tango_msgs::infrareds& irmsg)
{
  front_c_ir_msg.range = irmsg.front_c_ir.range;
  front_c_ir_msg.header.stamp = irmsg.front_c_ir.header.stamp;
  front_c_ir_msg.header.frame_id = irmsg.front_c_ir.header.frame_id;
  front_c_ir_msg.radiation_type = irmsg.front_c_ir.radiation_type;
  front_c_ir_msg.field_of_view = irmsg.front_c_ir.field_of_view;
  front_c_ir_msg.min_range = irmsg.front_c_ir.min_range;
  front_c_ir_msg.max_range = irmsg.front_c_ir.max_range;

  front_r_inner_ir_msg.range = irmsg.front_r_inner_ir.range;
  front_r_inner_ir_msg.header.stamp = irmsg.front_r_inner_ir.header.stamp;
  front_r_inner_ir_msg.header.frame_id = irmsg.front_r_inner_ir.header.frame_id;
  front_r_inner_ir_msg.radiation_type = irmsg.front_r_inner_ir.radiation_type;
  front_r_inner_ir_msg.field_of_view = irmsg.front_r_inner_ir.field_of_view;
  front_r_inner_ir_msg.min_range = irmsg.front_r_inner_ir.min_range;
  front_r_inner_ir_msg.max_range = irmsg.front_r_inner_ir.max_range;

  front_r_outer_ir_msg.range = irmsg.front_r_outer_ir.range;
  front_r_outer_ir_msg.header.stamp = irmsg.front_r_outer_ir.header.stamp;
  front_r_outer_ir_msg.header.frame_id = irmsg.front_r_outer_ir.header.frame_id;
  front_r_outer_ir_msg.radiation_type = irmsg.front_r_outer_ir.radiation_type;
  front_r_outer_ir_msg.field_of_view = irmsg.front_r_outer_ir.field_of_view;
  front_r_outer_ir_msg.min_range = irmsg.front_r_outer_ir.min_range;
  front_r_outer_ir_msg.max_range = irmsg.front_r_outer_ir.max_range;

  rear_r_ir_msg.range = irmsg.rear_r_ir.range;
  rear_r_ir_msg.header.stamp = irmsg.rear_r_ir.header.stamp;
  rear_r_ir_msg.header.frame_id = irmsg.rear_r_ir.header.frame_id;
  rear_r_ir_msg.radiation_type = irmsg.rear_r_ir.radiation_type;
  rear_r_ir_msg.field_of_view = irmsg.rear_r_ir.field_of_view;
  rear_r_ir_msg.min_range = irmsg.rear_r_ir.min_range;
  rear_r_ir_msg.max_range = irmsg.rear_r_ir.max_range;

  rear_l_ir_msg.range = irmsg.rear_l_ir.range;
  rear_l_ir_msg.header.stamp = irmsg.rear_l_ir.header.stamp;
  rear_l_ir_msg.header.frame_id = irmsg.rear_l_ir.header.frame_id;
  rear_l_ir_msg.radiation_type = irmsg.rear_l_ir.radiation_type;
  rear_l_ir_msg.field_of_view = irmsg.rear_l_ir.field_of_view;
  rear_l_ir_msg.min_range = irmsg.rear_l_ir.min_range;
  rear_l_ir_msg.max_range = irmsg.rear_l_ir.max_range;

  front_l_outer_ir_msg.range = irmsg.front_l_outer_ir.range;
  front_l_outer_ir_msg.header.stamp = irmsg.front_l_outer_ir.header.stamp;
  front_l_outer_ir_msg.header.frame_id = irmsg.front_l_outer_ir.header.frame_id;
  front_l_outer_ir_msg.radiation_type = irmsg.front_l_outer_ir.radiation_type;
  front_l_outer_ir_msg.field_of_view = irmsg.front_l_outer_ir.field_of_view;
  front_l_outer_ir_msg.min_range = irmsg.front_l_outer_ir.min_range;
  front_l_outer_ir_msg.max_range = irmsg.front_l_outer_ir.max_range;

  front_l_inner_ir_msg.range = irmsg.front_l_inner_ir.range;
  front_l_inner_ir_msg.header.stamp = irmsg.front_l_inner_ir.header.stamp;
  front_l_inner_ir_msg.header.frame_id = irmsg.front_l_inner_ir.header.frame_id;
  front_l_inner_ir_msg.radiation_type = irmsg.front_l_inner_ir.radiation_type;
  front_l_inner_ir_msg.field_of_view = irmsg.front_l_inner_ir.field_of_view;
  front_l_inner_ir_msg.min_range = irmsg.front_l_inner_ir.min_range;
  front_l_inner_ir_msg.max_range = irmsg.front_l_inner_ir.max_range;
}
	
int main(int argc, char **argv)
{
  ros::init(argc, argv, "infrared_sensors");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);   
  ros::Publisher front_c_ir_pub = n.advertise<sensor_msgs::Range>("/tango/ir_sensors/front_c_ir", 1000);
  ros::Publisher front_r_inner_ir_pub = n.advertise<sensor_msgs::Range>("/tango/ir_sensors/front_r_inner_ir", 1000);
  ros::Publisher front_r_outer_ir_pub = n.advertise<sensor_msgs::Range>("/tango/ir_sensors/front_r_outer_ir", 1000);
  ros::Publisher rear_r_ir_pub = n.advertise<sensor_msgs::Range>("/tango/ir_sensors/rear_r_ir", 1000);
  ros::Publisher rear_l_ir_pub = n.advertise<sensor_msgs::Range>("/tango/ir_sensors/rear_l_ir", 1000);
  ros::Publisher front_l_outer_ir_pub = n.advertise<sensor_msgs::Range>("/tango/ir_sensors/front_l_outer_ir", 1000);
  ros::Publisher front_l_inner_ir_pub = n.advertise<sensor_msgs::Range>("/tango/ir_sensors/front_l_inner_ir", 1000);
  ros::Subscriber ir_sub = n.subscribe("/infrareds", 1000, infraredsCallback);
  int count = 0;
  while (ros::ok())
  {
    front_c_ir_pub.publish(front_c_ir_msg);
    front_r_inner_ir_pub.publish(front_r_inner_ir_msg);
    front_r_outer_ir_pub.publish(front_r_outer_ir_msg);
    rear_r_ir_pub.publish(rear_r_ir_msg);
    rear_l_ir_pub.publish(rear_l_ir_msg);
    front_l_outer_ir_pub.publish(front_l_outer_ir_msg);
    front_l_inner_ir_pub.publish(front_l_inner_ir_msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  ros::spin();
  return 0;
}

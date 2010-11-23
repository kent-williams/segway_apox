#include <cstdio>
#include <math.h>
#include <unistd.h>
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/LaserScan.h"

using namespace ros;

bool read_scan(FILE *f, double *t, double *ranges)
{
  if (!f || feof(f))
    return false;
  double log_t, min_ang, max_ang, ang_inc, min_range, max_range;
  int nscan;
  if (8 != fscanf(f, "%lf %lf %lf %lf %lf %lf %lf %d", 
                  &log_t, t, &min_ang, &max_ang, &ang_inc, 
                  &min_range, &max_range, &nscan))
    return false;
  double dummy_reflectivity;
  for (int i = 0; i < nscan; i++)
    if (2 != fscanf(f, "%lf %lf", ranges+i, &dummy_reflectivity))
      return false;
  return true;
}

bool read_odom(FILE *f, double *t, double *x, double *y, double *yaw)
{
  if (4 != fscanf(f, "%lf %lf %lf %lf\n", t, x, y, yaw))
  {
    fprintf(stderr, "bad odom file format\n");
    return false;
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "segway_republisher");
  if (argc != 3)
  {
    printf("usage: republish_logs ODOMFILE HORIZ_SCAN\n");
    return 1;
  }
  ros::NodeHandle nh;
  ros::Publisher hscan_pub = nh.advertise<sensor_msgs::LaserScan>("horiz_scan",1);
	tf::TransformBroadcaster tf;
  double t = 0, x = 0, y = 0, yaw = 0;
  FILE *f_odom = fopen(argv[1],"r");
  FILE *f_hscan = fopen(argv[2],"r");
  if (!f_odom)
  {
    printf("couldn't open logfile: %s\n", argv[1]);
    return 1;
  }
  if (!read_odom(f_odom, &t, &x, &y, &yaw))
    return 1;
  for (int i = 0; i < 5500; i++)
    if (!read_odom(f_odom, &t, &x, &y, &yaw))
      return 1;

  ros::Duration time_offset = ros::Time::now() - ros::Time(t);
  double t_laser, ranges[181];
  do
  {
    if (!read_scan(f_hscan, &t_laser, ranges))
    {
      fprintf(stderr, "bogus laser log\n");
      return 1;
    }
  } while (t_laser < t && !feof(f_hscan));
  sensor_msgs::LaserScan scan_msg;
  scan_msg.header.frame_id = "horiz_scan";
  scan_msg.angle_min =  M_PI/2;
  scan_msg.angle_max = -M_PI/2;
  scan_msg.angle_increment = -M_PI/180;
  scan_msg.range_min = 0;
  scan_msg.range_max = 81;
  scan_msg.ranges.resize(181);

  while(ros::ok() && !feof(f_odom) && !feof(f_hscan))
  {
    ros::spinOnce();
    if (ros::Time::now() - time_offset > ros::Time(t))
    {
      btQuaternion q;
      q.setRPY(0, 0, yaw);
      tf.sendTransform(tf::StampedTransform(btTransform(q, btVector3(x, y, 0)),
                       ros::Time(t) + time_offset, "odom", "base_link"));
      if (!read_odom(f_odom, &t, &x, &y, &yaw))
        break;
    }
    while (ros::Time::now() - time_offset > ros::Time(t_laser))
    {
      btQuaternion q;
      q.setRPY(0,0,0);
      tf.sendTransform(tf::StampedTransform(btTransform(q, btVector3(0.25, 0, 0)),
                       ros::Time(t_laser) + time_offset, 
                       "base_footprint", "horiz_scan"));
      tf.sendTransform(tf::StampedTransform(btTransform(q, btVector3(0, 0, 0)),
                       ros::Time(t_laser) + time_offset, 
                       "base_link", "base_footprint"));
      scan_msg.scan_time = (ros::Time(t_laser) + time_offset).toSec();
      scan_msg.header.stamp = ros::Time(t_laser) + time_offset;
      for (int i = 0; i < 181; i++)
        scan_msg.ranges[i] = ranges[i];
      hscan_pub.publish(scan_msg);
      if (!read_scan(f_hscan, &t_laser, ranges))
        break;
    }
	  usleep(500);
  }
  fclose(f_odom);
  fclose(f_hscan);
  return 0;
}


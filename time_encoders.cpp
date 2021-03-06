#include <unistd.h>
#include <signal.h>
#include <cstdio>
#include "segway_apox.h"
#include "ros/time.h"

static bool g_done = false;
void ctrlc_handler(int)
{
  g_done = true;
}

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    fprintf(stderr, "usage: print_encoders DEVICE_OF_APOX_USBCAN\n");
    return 1;
  }
  SegwayApox segway(argv[1]);
  signal(SIGINT, ctrlc_handler);
  printf("ctrl-c to exit.\n");
  float x = 0, y = 0, yaw = 0;
  uint32_t left, right;
  ros::Time prev_time = ros::Time::now();
  for (int ms_count = 0; !g_done; ms_count++)
  {
    if (segway.poll(0.5)) 
    {
      segway.get_last_odometry(x, y, yaw, left, right); 
      ros::Time t = ros::Time::now();
      double delta = t.toSec() - prev_time.toSec();
      printf("%f (%f)\n", delta, 1.0 / delta);
      prev_time = t;
    }
    if (ms_count % 10 == 0)
      segway.send_vel_cmd(0, 0);
    usleep(1000);
  }
  printf("bye\n");
  return 0;
}


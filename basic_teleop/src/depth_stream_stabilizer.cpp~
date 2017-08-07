#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <sstream>
//#include <thread> multiple threads may not be supported
sensor_msgs::Image lastReceivedFrame;

void callback(sensor_msgs::Image frame) {
    lastReceivedFrame = frame;
}

/*
void run_subscriber(ros::NodeHandle * n) { // intended for threaddedness
    ros::Subscriber sub = n->subscribe("/openni2_camera/depth/image_raw", 1000, callback);
    ros::spin();
}
*/

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "depth_restreamer");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<sensor_msgs::Image>("depth_stabilized", 1000);
  ros::Subscriber sub = n.subscribe("/openni2_camera/depth/image_raw", 1000, callback);

  ros::Rate loop_rate(60); // 60 fps

  /**
   * A count of how many messages we have sent.
   */
  int count = 0;
  while (ros::ok())
  {

    if (&lastReceivedFrame != NULL){ // make sure at least one frame has been recieved
        chatter_pub.publish(lastReceivedFrame);
    }

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}

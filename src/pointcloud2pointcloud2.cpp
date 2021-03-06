#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "laser_geometry/laser_geometry.h"

ros::Publisher cloud_pub;

void scanCallback (const sensor_msgs::PointCloud::ConstPtr& scan_in)
{
	sensor_msgs::PointCloud2 cloud;
  if (sensor_msgs::convertPointCloudToPointCloud2(*scan_in, cloud))
		cloud_pub.publish(cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointCloudToPointCloud2");
  ros::NodeHandle n;
  ros::Subscriber subWaypoint = n.subscribe("base_scan", 1000, scanCallback);
	cloud_pub = n.advertise<sensor_msgs::PointCloud2>("cloud_in", 1000);
  ros::spin();
  return 0;
}

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/cloud_viewer.h>


pcl::VoxelGrid<pcl::PointXYZ> sor;
pcl::PassThrough<pcl::PointXYZ> pass_through;

void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{	 
	  pcl::PointCloud <pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());

	  // Convert to PCL data type
	  pcl::fromROSMsg(*cloud_msg, *cloud);

	  
	  // Filter
	  
	  pass_through.setInputCloud (cloud);
	  pass_through.setFilterLimits (0.5, 10);
	  pass_through.setFilterFieldName ("z");
	  pass_through.filter(*cloud);

	  
	  // Voxel
	  
	  sor.setInputCloud (cloud);
	  sor.setLeafSize (0.1, 0.1, 0.1);
      sor.filter (*cloud);
      
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      
viewer->setBackgroundColor (0, 0, 0);
viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
viewer->initCameraParameters ();
while (!viewer->wasStopped ())
{
  viewer->spinOnce (1);
  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
}
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointCloud");
  
   ros::NodeHandle n;
   
  ros::Subscriber sub = n.subscribe("/camera/depth/points", 10, callback);

  ros::spin();
  return 0;
}

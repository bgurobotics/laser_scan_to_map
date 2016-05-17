#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <iostream>
#include "opencv2/core/core.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define proportion_size 0.05

using namespace cv;
using namespace std;

//globals for the laser scanner:
float angle_min ;       // start angle of the scan [rad]
float angle_max  ;      // end angle of the scan [rad]
float angle_increment ; // angular distance between measurements [rad]
float time_increment ;  // time between measurements [seconds] - if your scanner
                         // is moving, this will be used in interpolating position
                         // of 3d points
float scan_time  ;      // time between scans [seconds]
float range_min  ;      // minimum range value [m]
float range_max  ;      // maximum range value [m]
std::vector<float> ranges(1);
size_t cell_length;
bool is_first_run = true;
//globals from the odometry node:
float x_global, y_global, theta_global;
std::vector<float> x_range(1);
std::vector<float> y_range(1);


void position_callback(const geometry_msgs::Pose &msg)
{
x_global=msg.position.x;
y_global=msg.position.y;
theta_global= msg.orientation.z;
	if(!is_first_run)
	{
	size_t i;
		for (i=0;i<ranges.size();i++)
		{	
		float theta = angle_min+i*angle_increment;
			if(ranges[i]<angle_max && ranges[i]>angle_min)
			{
			x_range[i]=x_global+(cos(theta)*cos(theta_global)-sin(theta)*sin(theta_global))*ranges[i];
			y_range[i]=y_global+(cos(theta)*sin(theta_global)+sin(theta)*cos(theta_global))*ranges[i];
			}
			else
			{
			x_range[i]=1000; 
			y_range[i]=1000;
			}
	    }
	}
	
} 

void laser_scan_callback(const sensor_msgs::LaserScan &msg)
{
	if(is_first_run)
	{
	angle_min=msg.angle_min;
	angle_max=msg.angle_max;
	angle_increment=msg.angle_increment;
	range_min=msg.range_min;
	range_max=msg.range_max;
	time_increment=msg.time_increment;
	cell_length=(angle_max-angle_min)/angle_increment;
	ranges.resize(cell_length);
	x_range.resize(cell_length);
	y_range.resize(cell_length);
	is_first_run=false;
	}
	else
	{
	scan_time=msg.scan_time;
	size_t i;
		for (i=0;i<ranges.size();i++)
		{	
		ranges[i]=msg.ranges[i];
		}
	}
}

void update_obstacle_map(cv::Mat &obstacle_map)
{
	size_t i;
		if(!is_first_run)
		{
			for (i=0;i<ranges.size();i++)
			{	
			obstacle_map.at<uchar>((int)(x_range[i]/proportion_size),(int)(y_range[i]/proportion_size))=1;
			}
		}
	
}


int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan_to_map");
  ros::NodeHandle n;
  ros::Subscriber laser_scan_sub = n.subscribe("/laser_scan", 1000,laser_scan_callback);
  ros::Subscriber position_sub = n.subscribe("/rosbot/position", 1000,position_callback);
  ros::Rate loop_rate(30);
  image_transport::ImageTransport it(n);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  sensor_msgs::ImagePtr obstacle_map_msg;
  Mat obstacle_map = cv::Mat(100,100, CV_8UC1,255);
  
  while(n.ok())
  {
	update_obstacle_map(obstacle_map); 
	obstacle_map_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", obstacle_map).toImageMsg();
	pub.publish(obstacle_map_msg);
	loop_rate.sleep();  
	ros::spinOnce();  
  }
}

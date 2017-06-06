#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <string>
#include <sstream>
#include <math.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui.hpp>

#define SCALE  50
#define WIDTH  1280
#define HEIGHT 1024

void distanceCallback(const sensor_msgs::LaserScan::ConstPtr& msg, ros::Publisher pub) {
	const double pi = 3.1415926535897;
	float min = msg->ranges[0];
	double angr, angr2;
	int xo, yo, x1, x2, y1, y2;
	
	std::string txt;
	std::stringstream ss, msgss;
	std_msgs::String cbmsg;	
	
	cv::Point p1, p2, po;	
	cv::Mat O(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(255,255,255));
	
	//Centre coordinates
	xo = WIDTH/2;
	yo = HEIGHT/2;
	po = cv::Point(WIDTH/2, HEIGHT/2);
	
	//Draw the centre
	cv::line(O, po, po, cv::Scalar(0, 0, 0), 2);
	
	int len = msg->ranges.size();
	for(int i=0; i<len-1; ++i) {
		
		//Transforming angle from radians to degrees
		angr = i*(msg->angle_increment) + (msg->angle_min);
		
		//Coordinates for the first point
		x1 = WIDTH/2 - SCALE*(msg->ranges[i])*cos(angr);
		y1 = SCALE*(msg->ranges[i])*sin(angr) + HEIGHT/2;
		p1 = cv::Point(x1, y1);
		
		//Coordinates for the second point
		x2 = WIDTH/2 - SCALE*(msg->ranges[i+1])*cos(angr);
		y2 = SCALE*(msg->ranges[i+1])*sin(angr) + HEIGHT/2;
		p2 = cv::Point(x2, y2);
		
		//Connect points
		cv::line(O, p1, p2, cv::Scalar(0, 0, 0), 2);
		
		//Check if the current point is closer to the robot than the current min
		if(msg->ranges[i] < msg->ranges[min])
			min = i;
	}
	
	//Closest obstacle
	x1 = WIDTH/2 - SCALE*(msg->ranges[min])*cos((min*(msg->angle_increment) + msg->angle_min));
	y1 = SCALE*(msg->ranges[min])*sin((min*(msg->angle_increment) + msg->angle_min)) + HEIGHT/2;	
	p1 = cv::Point(x1, y1);
	
	//Publish message
	msgss << "Closest obstacle at " << msg->ranges[min];
	cbmsg.data = msgss.str();
	pub.publish(cbmsg);
	
	//Line from robot to closest obstacle
	cv::line(O, p1, po, cv::Scalar(255, 175, 190), 2);
	
	//Circle around closest obstacle
	cv::circle(O, p1, 32.0, cv::Scalar(0, 0, 255), 1);
	
	//Distance text origin point
	x2 = (x1 + xo)/2 + SCALE/10;
	y2 = (y1 + yo)/2 + SCALE/10;
	p2 = cv::Point(x2, y2);
	
	//Wtite distance
	ss << "[" << msg->ranges[min] << "m]";	
	txt = ss.str();
	cv::putText(O, txt, p2, cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0));
	
	//Draww stuff
	cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
	cv::imshow("Image", O);	
	cv::waitKey(2);
	
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "minnode_img");
	tf::TransformListener tl;
	
	ros::NodeHandle n;
	
	ros::Publisher pub = n.advertise<std_msgs::String>("mintopic_img", 1000);
	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("base_scan", 1000, boost::bind(distanceCallback, _1, pub));
	
	ros::spin();
	return 0;
}

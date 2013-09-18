#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <image_transport/image_transport.h>
#include <stdio.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <sensor_msgs/image_encodings.h>

#include "CamGrabber.hpp"


using namespace cv;
using namespace std;

/****** SETTINGS ************/
#define CAPTURE_SOURCE 		"/dev/video0"
#define CAPTURE_HEIGHT 		360
#define CAPTURE_WIDTH  		640
#define FPS		    	10
/****************************/


/* captured image */
Mat frame(CAPTURE_HEIGHT,CAPTURE_WIDTH,CV_8UC1);


/* video capture instance */
CamGrabber webcam(CAPTURE_SOURCE, CAPTURE_WIDTH, CAPTURE_HEIGHT);

void listTopics(void)
{
	//  print published/subscribed topics
	ros::V_string topics;
	ros::this_node::getSubscribedTopics(topics);
	std::string nodeName = ros::this_node::getName();
	std::string topicsStr = nodeName + ":\n\tsubscribed to topics:\n";
	for(unsigned int i=0; i<topics.size(); i++)
	topicsStr+=("\t\t" + topics.at(i) + "\n");

	topicsStr += "\tadvertised topics:\n";
	ros::this_node::getAdvertisedTopics(topics);
	for(unsigned int i=0; i<topics.size(); i++)
	topicsStr+=("\t\t" + topics.at(i) + "\n");

	ROS_INFO_STREAM("\n" << topicsStr);
}


int main(int argc, char * argv[])
{
	/* INIT ROS */
	ros::init(argc, argv, "coax_cam");
	ros::NodeHandle node;


	/*************************/
	/* ROS Messages  		 */
	/*************************/

	/* Publish */
	image_transport::ImageTransport image_trsp(node);
	image_transport::Publisher image_pub = image_trsp.advertise("cv_capture", 1);


	listTopics();


	/* Work loop */
	int i=0;
	ros::Rate rate(FPS);


	while(node.ok())
	{

		/* get the frame */
		webcam.grabFrame(&frame.data, true);

		/* publish to ROS */
		cv_bridge::CvImage out_msg;
		out_msg.encoding = sensor_msgs::image_encodings::MONO8;
		out_msg.image    = frame;
		out_msg.header.stamp = ros::Time::now();

		/* send compressed */
		image_pub.publish(out_msg.toImageMsg());

		/* throttle loop*/
		if(!rate.sleep())
			ROS_WARN_THROTTLE(1,"Loop too slow for desired FPS!");
		ros::spinOnce();
	}

	return 0;
}

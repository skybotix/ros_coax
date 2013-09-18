#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "coax_msgs/CoaxState.h"

#include <boost/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class CoaxPTAMInterface
{
	protected:
		ros::Publisher pose_pub;
		ros::Subscriber state_sub;
		image_transport::Subscriber camera_sub;

		coax_msgs::CoaxState latestState;
		boost::mutex image_mutex;


	public:
		CoaxPTAMInterface(ros::NodeHandle & n) 
		{
			// Publishing the position
			pose_pub = n.advertise<geometry_msgs::PoseStamped>("coax_3d/pose",10);

			// Subscribing to state message 
			ros::TransportHints hints;
			state_sub = n.subscribe("coax_server/state",10,&CoaxPTAMInterface::stateCallback,this,hints.udp());

			std::string transport = "theora";
			image_transport::ImageTransport it(n);
			// We use a queue of only one image (the '1' below). Other image
			// are discarded
			camera_sub = it.subscribe("/cv_capture", 1, &CoaxPTAMInterface::imageCallback, this, transport);

		}

		~CoaxPTAMInterface() {
		}

		void stateCallback(const coax_msgs::CoaxState::ConstPtr & message) {
			latestState = *message;
		}

		void imageCallback(const sensor_msgs::ImageConstPtr& msg)
		{
        // Mutex to avoid being called back twice
        boost::lock_guard<boost::mutex> guard(image_mutex);

        // Convert image from ROS msg to OpenCV
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            // Conversion
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

            // Show image
            cv::namedWindow("CoaX Video", CV_WINDOW_AUTOSIZE);
            cv::startWindowThread();
            cv::imshow("CoaX Video", cv_ptr->image);
            cv::waitKey(3);


            // Here do some processing on OpenCV image


            // Now prepare the pose and publish it
            geometry_msgs::PoseStamped pose;
            // A time stamp in the pose
            pose.header.stamp = ros::Time::now();
            // Provide absolute position
            pose.pose.position.x = 0; // TODO
            pose.pose.position.y = 0; // TODO
            pose.pose.position.z = 0; // TODO
            // Orientation is represented as a quaternion
            pose.pose.orientation.x = 0; // TODO
            pose.pose.orientation.y = 0; // TODO
            pose.pose.orientation.z = 1; // TODO
            pose.pose.orientation.w = 1; // TODO
            pose_pub.publish(pose);


        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Unable to convert %s image to bgr8! cv_bridge exception: %s", msg->encoding.c_str(), e.what());
            return;
        }
    }

};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "coax_ptam_interface");
	ros::NodeHandle n;

	CoaxPTAMInterface api(n);


	ROS_INFO("Coax PTAM Interface");

	ros::spin();

	return 0;
}


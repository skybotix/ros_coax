#ifndef COAX_GUI_HANDLER_H
#define COAX_GUI_HANDLER_H


#include <QObject>
#include <QDesktopWidget>
#include <QApplication>
#include <QtGui>
#include <QtGui/QMainWindow>

#include <boost/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "coax_msgs/CoaxState.h"
#include "ui_CoaxGUI.h"

class CoaxGUIHandler : public QMainWindow {
		Q_OBJECT
	protected:
		Ui_CoaxGUI gui;
		QImage video;
		QTimer controlTimer, guiTimer;
		ros::AsyncSpinner spinner;

		coax_msgs::CoaxState state,cfgstate;
		ros::Subscriber state_sub;
		ros::Publisher control_pub;
		ros::ServiceClient cfgControlClt;
		ros::ServiceClient cfgCommClt;
		ros::ServiceClient cfgOAClt;
		ros::ServiceClient reachNavStateClt;
		ros::ServiceClient setTimeoutClt;
		image_transport::Subscriber camera_sub;

		boost::mutex image_mutex;



		bool configureCoaX;
		bool manualControl;
		float desiredRoll, desiredPitch, desiredYaw, desiredAlt;

	public:
		CoaxGUIHandler(ros::NodeHandle & n);
		~CoaxGUIHandler();

		void stateCallback(const coax_msgs::CoaxState::ConstPtr& msg) {
			// printf("Got State\n");
			state = *msg;
		}

		void imageCallback(const sensor_msgs::ImageConstPtr& msg)
		{
			boost::lock_guard<boost::mutex> guard(image_mutex);

      // Convert image from ROS msg to OpenCV
      cv_bridge::CvImagePtr cv_ptr;

      try
      {
          // Conversion
          cv_ptr = cv_bridge::toCvCopy(msg, "mono8");

          // Show image (debug)
          //cv::namedWindow("CoaX Video", CV_WINDOW_AUTOSIZE);
          //cv::startWindowThread();
          //cv::imshow("CoaX Video", cv_ptr->image);
          //cv::waitKey(3);

          video = QImage((const uchar*)(cv_ptr->image.data), cv_ptr->image.cols, cv_ptr->image.rows, QImage::Format_Indexed8);

      }
      catch (cv_bridge::Exception& e)
      {
          ROS_ERROR("Unable to convert %s image to mono8! cv_bridge exception: %s", msg->encoding.c_str(), e.what());
          return;
      }


		}

		/**
		 * Examine the key pressed and move the disc accordingly.
		 **/
		void keyPressEvent (QKeyEvent *);

	public slots:
		void updateGui(); 
		void updateControl();

		void prepareConfiguration();
		void cancelConfiguration();
		void applyConfiguration();

		void toggleKeyControl(bool state);
};


#endif // COAX_GUI_HANDLER_H

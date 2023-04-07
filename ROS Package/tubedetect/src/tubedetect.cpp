/*
 * tubedetect.cpp
 *
 *  Created on: Nov 14, 2012
 *      Author: mende
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "image_transport/image_transport.h"

#include "tubedetect.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

#include "imageprocessing.h"

#include <dynamic_reconfigure/server.h>
//#include <tubedetect/MyStuffConfig.h>
#include <tubedetect/tubedetect_paramsConfig.h>

ImageProcessing Processor;
int downH=38;
int upH=70;
int downS=40;
int upS=256;
int downV=10;
int upV=240;

ros::Publisher tubedetect_pub_dummy;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  sensor_msgs::CvBridge bridge;


  try
  {
	  cv::Mat image(bridge.imgMsgToCv(msg, "bgr8"));

	  image=Processor.ProcessImage(image);

	  cv::imshow("view", image);
	  cv::waitKey(1);
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}








int main(int argc, char **argv)
{
	  /**
	   * The ros::init() function needs to see argc and argv so that it can perform
	   * any ROS arguments and name remapping that were provided at the command line. For programmatic
	   * remappings you can use a different version of init() which takes remappings
	   * directly, but for most command-line programs, passing argc and argv is the easiest
	   * way to do it.  The third argument to init() is the name of the node.
	   *
	   * You must call one of the versions of ros::init() before using any other
	   * part of the ROS system.
	   */
	  ros::init(argc, argv, "TubeDetect");

	  /**
	   * NodeHandle is the main access point to communications with the ROS system.
	   * The first NodeHandle constructed will fully initialize this node, and the last
	   * NodeHandle destructed will close down the node.
	   */
	  ros::NodeHandle n;


	  /**
	     * The advertise() function is how you tell ROS that you want to
	     * publish on a given topic name. This invokes a call to the ROS
	     * master node, which keeps a registry of who is publishing and who
	     * is subscribing. After this advertise() call is made, the master
	     * node will notify anyone who is trying to subscribe to this topic name,
	     * and they will in turn negotiate a peer-to-peer connection with this
	     * node.  advertise() returns a Publisher object which allows you to
	     * publish messages on that topic through a call to publish().  Once
	     * all copies of the returned Publisher object are destroyed, the topic
	     * will be automatically unadvertised.
	     *
	     * The second parameter to advertise() is the size of the message queue
	     * used for publishing messages.  If messages are published more quickly
	     * than we can send them, the number here specifies how many messages to
	     * buffer up before throwing some away.
	     */
	    ros::Publisher tubedetect_pub = n.advertise<std_msgs::String>("tubedetect", 1000);
	    tubedetect_pub_dummy = n.advertise<std_msgs::Float32>("tubedetect_dummy", 1000);

	    // The (very low) loop rate is only necessary to detect configuration changes
	    ros::Rate loop_rate(50);

	    // images are sent via image transport package, see http://www.ros.org/wiki/image_transport for details
	    image_transport::ImageTransport it(n);
	    image_transport::CameraPublisher pub = it.advertiseCamera("tubedetect/image", 1);
	    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);


	    // Set up a dynamic reconfigure server.
	     // This should be done before reading parameter server values.
	     dynamic_reconfigure::Server<tubedetect::tubedetect_paramsConfig> dr_srv;
	     dynamic_reconfigure::Server<tubedetect::tubedetect_paramsConfig>::CallbackType cb;
	     cb = boost::bind(&ImageProcessing::configCallback, Processor, _1, _2);
	     dr_srv.setCallback(cb);


	     // Initialize node parameters from launch file or command line.
	     // Use a private node handle so that multiple instances of the node can
	     // be run simultaneously while using different parameters.
	     ros::NodeHandle private_node_handle_("~");
//	     private_node_handle_.param("lower_bound_H", ::downH, int(1));
//     	 private_node_handle_.param("upper_bound_H", ::upH, int(2));


		ROS_INFO("TubeDetect starts processing");

		cv::namedWindow("view",CV_WINDOW_NORMAL|CV_WINDOW_KEEPRATIO);
//		cv::namedWindow("step1",CV_WINDOW_NORMAL|CV_WINDOW_KEEPRATIO);
		cv::namedWindow("step2",CV_WINDOW_NORMAL|CV_WINDOW_KEEPRATIO);

		cv::waitKey(1);

		while (ros::ok())
		{
			ros::spinOnce();

			loop_rate.sleep();
		}

		cv::destroyWindow("view");

		return 0;

}





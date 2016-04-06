/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/


#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/agilus_master_project/qnode.hpp"


namespace agilus_master_project {


QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
    init_argv(argv),
    cloud(new pcl::PointCloud<pcl::PointXYZ>),
    imageReading(false)
    {
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZ>::Ptr >("pcl::PointCloud<pcl::PointXYZ>::Ptr");
    qRegisterMetaType<cv::Mat>("cv::Mat");
}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"agilus_master_project");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.

	start();
	return true;
}

void QNode::run() {
    ros::Rate loop_rate(60);
	while ( ros::ok() ) {

		ros::spinOnce();
		loop_rate.sleep();
	}
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::subscribeToPointCloud2(QString topic)
{
    ros::NodeHandle n;
    const char *tmp = topic.toUtf8().constData();
    pointCloud2Sub = n.subscribe<sensor_msgs::PointCloud2, QNode>(tmp, 1, &QNode::cloudCallback,this);

}

void QNode::subscribeTo2DobjectDetected(QString topic)
{
    ros::NodeHandle n;
    const char *tmp = topic.toUtf8().constData();
    object2DdetectedSub = n.subscribe<sensor_msgs::Image, QNode>(tmp, 1, &QNode::object2DCallback,this);
}



void QNode::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){
    // Convert ROS message (PointCloud2) to PCL point cloud (PointCloud(PointXYZ))
    pcl::fromROSMsg(*cloud_msg, *cloud);
    Q_EMIT updatePointCloud(cloud);
}

void QNode::object2DCallback(const sensor_msgs::ImageConstPtr &image)
{
    cv_bridge::CvImagePtr cv_ptr;

    // Early return if we are still reading the image
    if(imageReading){
        std::cout << "Not done" << std::endl;
        return;
    }


    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, "8UC3");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    std::cout << "Got image" << std::endl;

    // Emitting update image, remember to set the flag
    setImageReading(true);
    image2d = cv_ptr->image;
    Q_EMIT update2Dimage(image2d);
}

bool QNode::getImageReading()
{
    return imageReading;
}

void QNode::setImageReading(bool reading)
{
    this->imageReading = reading;
}

}  // namespace agilus_master_project

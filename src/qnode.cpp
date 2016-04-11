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
    setImageprocessorRunning = n.serviceClient<image_processor::setProcessRunning>("/object_2D_detection/setProcessRunning");
    setImageprocessorColor = n.serviceClient<image_processor::setVideoColor>("/object_2D_detection/setVideoColor");
    setImageprocessorUndistort = n.serviceClient<image_processor::setVideoUndistortion>("/object_2D_detection/setVideoUndistortion");
    setImageprocessorBruteforce = n.serviceClient<image_processor::setBruteforceMatching>("/object_2D_detection/setBruteforceMatching");
    setImageprocessorKeypoint = n.serviceClient<image_processor::setKeypointDetectorType>("/object_2D_detection/setKeypointDetectorType");
    setImageprocessorDescriptor = n.serviceClient<image_processor::setDescriptorType>("/object_2D_detection/setDescriptorType");
    setImageprocessorMatchingImage = n.serviceClient<image_processor::setMatchingImage1>("object_2D_detection/setMatchingImage1");

    goToClient_ag1 = n.serviceClient<agilus_planner::Pose>("/robot_service_ag1/go_to_pose");
    goToClient_ag2 = n.serviceClient<agilus_planner::Pose>("/robot_service_ag2/go_to_pose");
    planClient_ag1 = n.serviceClient<agilus_planner::Pose>("/robot_service_ag1/plan_pose");
    planClient_ag2 = n.serviceClient<agilus_planner::Pose>("/robot_service_ag2/plan_pose");

    object2Dpose = n.subscribe<geometry_msgs::Pose2D,QNode>("/object_2D_detected/object1", 1000, &QNode::object2DPoseCallback,this);

    plan_ag1(0.445,-0.6025,1.66,0.0,3.1415,0.0);
    plan_ag1(0.445,-0.6025,1.66,0.0,3.1415,0.0);
    plan_ag2(0.445,0.6025,1.66,0.0,3.1415,0.0);
    plan_ag2(0.445,0.6025,1.66,0.0,3.1415,0.0);

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

void QNode::setProcessImageRunning(bool processRunning)
{
    setImproRunning.request.running = processRunning;
    setImageprocessorRunning.call(setImproRunning);
}

void QNode::setProcessImageColor(bool color)
{
    setVideoColor.request.color = color;
    setImageprocessorColor.call(setVideoColor);
}

void QNode::setProcessImageUndistort(bool undistort)
{
    setVideoUndist.request.undistort = undistort;
    setImageprocessorUndistort.call(setVideoUndist);
}

void QNode::setProcessImageBruteforce(bool bruteforce)
{
    setBF.request.bruteforce = bruteforce;
    setImageprocessorBruteforce.call(setBF);
}

void QNode::setProcessImageKeypointDescriptor(std::string keypoint, std::string descriptor)
{
    setKeypoint.request.type = keypoint;
    setDescriptor.request.type = descriptor;
    setImageprocessorKeypoint.call(setKeypoint);
    setImageprocessorDescriptor.call(setDescriptor);
}

void QNode::setProcessImageMatchingPicture(std::string imagePath)
{
    setMatchingImage.request.imagePath = imagePath;
    setImageprocessorMatchingImage.call(setMatchingImage);
}

void QNode::move_ag1(double x, double y, double z, double roll, double pitch, double yaw)
{
    setPoseRequest(false,true,x,y,z,true,roll,pitch,yaw);
    goToClient_ag1.call(pose_service);
}

void QNode::move_ag2(double x, double y, double z, double roll, double pitch, double yaw)
{
    setPoseRequest(false,true,x,y,z,true,roll,pitch,yaw);
    goToClient_ag2.call(pose_service);
}

void QNode::plan_ag1(double x, double y, double z, double roll, double pitch, double yaw)
{
    setPoseRequest(false,true,x,y,z,true,roll,pitch,yaw);
    planClient_ag1.call(pose_service);
}

void QNode::plan_ag2(double x, double y, double z, double roll, double pitch, double yaw)
{
    setPoseRequest(false,true,x,y,z,true,roll,pitch,yaw);
    planClient_ag2.call(pose_service);
}

void QNode::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){
    // Convert ROS message (PointCloud2) to PCL point cloud (PointCloud(PointXYZ))
    pcl::fromROSMsg(*cloud_msg, *cloud);
    Q_EMIT updatePointCloud(cloud);
}

void QNode::object2DCallback(const sensor_msgs::ImageConstPtr &image)
{
    // Early return if we are still reading the image
    if(imageReading){
        return;
    }
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, "8UC3");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Emitting update image, remember to set the flag
    setImageReading(true);
    Q_EMIT update2Dimage(mat2qimage(cv_ptr->image));
}

void QNode::object2DPoseCallback(const geometry_msgs::Pose2DConstPtr &msg)
{
    x_object = msg->x;
    y_object = msg->y;
}

QImage QNode::mat2qimage(cv::Mat& mat) {
    switch (mat.type()) {
        // 8-bit, 4 channel
        case CV_8UC4: {
            QImage image(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB32);
            return image;
        }

        // 8-bit, 3 channel
        case CV_8UC3: {
            const uchar *qImageBuffer = (const uchar*)mat.data;
            QImage image(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
            return image.rgbSwapped();
        }

        // 8-bit, 1 channel
        case CV_8UC1: {
            static QVector<QRgb>  sColorTable;

            // only create our color table once
            if (sColorTable.isEmpty()) {
                for (int i = 0; i < 256; ++i)
                    sColorTable.push_back(qRgb(i, i, i));
            }

            QImage image(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);
            image.setColorTable(sColorTable);

            return image;
        }

        default:
            std::cout << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << mat.type();
            break;
    }

    return QImage();
}

std::string QNode::type2str(int type) {
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth) {
        case CV_8U:
            r = "8U";
            break;
        case CV_8S:
            r = "8S";
            break;
        case CV_16U:
            r = "16U";
            break;
        case CV_16S:
            r = "16S";
            break;
        case CV_32S:
            r = "32S";
            break;
        case CV_32F:
            r = "32F";
            break;
        case CV_64F:
            r = "64F";
            break;
        default:
            r = "User";
            break;
    }

    r += "C";
    r += (chans + '0');

    // USAGE
    //    std::string ty =  type2str( H.type() );
    //    printf("Matrix: %s %dx%d \n", ty.c_str(), H.cols, H.rows );

    return r;
}

void QNode::setPoseRequest(bool relative, bool position, double x, double y, double z, bool orientation, double roll, double pitch, double yaw)
{
    pose_service.request.header.frame_id = "/world";
    pose_service.request.relative = relative;
    pose_service.request.set_position = position;
    pose_service.request.position_x = x;
    pose_service.request.position_y = y;
    pose_service.request.position_z = z;
    pose_service.request.set_orientation = orientation;
    pose_service.request.orientation_r = roll;
    pose_service.request.orientation_p = pitch;
    pose_service.request.orientation_y = yaw;
}

double QNode::getXoffset()
{
    return x_object;
}

double QNode::getYoffset()
{
    return y_object;
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

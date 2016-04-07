/**
 * @file /include/agilus_master_project/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef agilus_master_project_QNODE_HPP_
#define agilus_master_project_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/atomic.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <image_processor/getBruteforceMatching.h>
#include <image_processor/setBruteforceMatching.h>
#include <image_processor/getProcessRunning.h>
#include <image_processor/setProcessRunning.h>
#include <image_processor/getVideoColor.h>
#include <image_processor/setVideoColor.h>
#include <image_processor/getVideoUndistortion.h>
#include <image_processor/setVideoUndistortion.h>
#include <image_processor/getKeypointDetectorType.h>
#include <image_processor/setKeypointDetectorType.h>
#include <image_processor/getDescriptorType.h>
#include <image_processor/setDescriptorType.h>
#include <image_processor/setMatchingImage1.h>

#include <agilus_planner/Pose.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace agilus_master_project {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

	QStringListModel* loggingModel() { return &logging_model; }
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void object2DCallback(const sensor_msgs::ImageConstPtr &image);
    bool getImageReading();
    void setImageReading(bool reading);
    QImage mat2qimage(cv::Mat& mat);
    std::string type2str(int type);
    void setPoseRequest(bool relative, bool position,
                        double x, double y, double z,
                        bool orientation, double roll, double pitch,
                        double yaw);

public Q_SLOTS:
    void subscribeToPointCloud2(QString topic);
    void subscribeTo2DobjectDetected(QString topic);
    void setProcessImageRunning(bool processRunning);
    void setProcessImageColor(bool color);
    void setProcessImageUndistort(bool undistort);
    void setProcessImageBruteforce(bool bruteforce);
    void setProcessImageKeypointDescriptor(std::string keypoint, std::string descriptor);
    void setProcessImageMatchingPicture(std::string imagePath);
    void move_ag1(double x, double y, double z,
                  double roll, double pitch, double yaw);
    void move_ag2(double x, double y, double z,
                  double roll, double pitch, double yaw);
    void plan_ag1(double x, double y, double z,
                  double roll, double pitch, double yaw);
    void plan_ag2(double x, double y, double z,
                  double roll, double pitch, double yaw);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void updatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void update2Dimage(QImage image);

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;
    ros::Subscriber pointCloud2Sub;
    ros::Subscriber object2DdetectedSub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    boost::atomic_bool imageReading;

    // Image processor
    ros::ServiceClient getImageprocessorRunning;
    ros::ServiceClient setImageprocessorRunning;
    ros::ServiceClient getImageprocessorColor;
    ros::ServiceClient setImageprocessorColor;
    ros::ServiceClient getImageprocessorBruteforce;
    ros::ServiceClient setImageprocessorBruteforce;
    ros::ServiceClient getImageprocessorUndistort;
    ros::ServiceClient setImageprocessorUndistort;
    ros::ServiceClient getImageprocessorKeypoint;
    ros::ServiceClient setImageprocessorKeypoint;
    ros::ServiceClient getImageprocessorDescriptor;
    ros::ServiceClient setImageprocessorDescriptor;
    ros::ServiceClient setImageprocessorMatchingImage;

    image_processor::getProcessRunning getImproRunning;
    image_processor::setProcessRunning setImproRunning;
    image_processor::getVideoColor getVideoColor;
    image_processor::setVideoColor setVideoColor;
    image_processor::getVideoUndistortion getVideoUndist;
    image_processor::setVideoUndistortion setVideoUndist;
    image_processor::getBruteforceMatching getBF;
    image_processor::setBruteforceMatching setBF;
    image_processor::getKeypointDetectorType getKeypoint;
    image_processor::setKeypointDetectorType setKeypoint;
    image_processor::getDescriptorType getDescriptor;
    image_processor::setDescriptorType setDescriptor;
    image_processor::setMatchingImage1 setMatchingImage;

    // Robot control
    ros::ServiceClient goToClient_ag1;
    ros::ServiceClient goToClient_ag2;
    ros::ServiceClient planClient_ag1;
    ros::ServiceClient planClient_ag2;
    agilus_planner::Pose pose_service;
};

}  // namespace agilus_master_project

#endif /* agilus_master_project_QNODE_HPP_ */

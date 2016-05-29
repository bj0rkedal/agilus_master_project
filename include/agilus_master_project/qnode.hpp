/**
 * @file /include/agilus_master_project/qnode.hpp
 *
 * @brief Original author Asgeir Bjoerkedal & Kristoffer Larsen. Handles all ROS communication.
 *
 * @date May 2016
 **/

#ifndef agilus_master_project_QNODE_HPP_
#define agilus_master_project_QNODE_HPP_

#include <ros/ros.h>
#include <ros/package.h>
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
#include <geometry_msgs/Pose2D.h>
#include <boost/atomic.hpp>

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
#include <image_processor/setImageDepth.h>

#include <moveit/move_group_interface/move_group.h>
#include <kuka_rsi_hw_interface/write_8_outputs.h>
#include <agilus_planner/Pose.h>

namespace agilus_master_project {

class QNode : public QThread {
    Q_OBJECT
public:

    /*!
     * \brief Constructor for the QNode class.
     * \param argc Initial argument.
     * \param argv Initial argument.
     */
	QNode(int argc, char** argv );

    /*!
     * \brief Desconstructor for the QNode class.
     */
	virtual ~QNode();

    /*!
     * \brief Initiates the ROS communication.
     * \return False if ROS initial setup failed.
     */
	bool init();

    /*!
     * \brief Starts the main thread. At this point, the thread is driven by ROS communication events.
     */
	void run();

    /*!
     * \brief Callback method called when a new 3D point cloud is published.
     * \param cloud_msg The new 3D point cloud.
     */
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

    /*!
     * \brief Callback method called when a new 2D image is published.
     * \param image The new 2D image.
     */
    void object2DCallback(const sensor_msgs::ImageConstPtr &image);

    /*!
     * \brief Callback method called when a new object pose is detected by the 2D object detection process in the image processor.
     * \param msg The new object pose.
     */
    void object2DPoseCallback(const geometry_msgs::Pose2DConstPtr &msg);

    /*!
     * \brief Returns true if a 2D image is currently being read.
     * \return True if a 2D image is currently being read.
     */
    bool getImageReading();

    /*!
     * \brief Set the flag used to lock the image object read from the 2D video feed topic.
     * \param reading Set true when the reading starts.
     */
    void setImageReading(bool reading);

    /*!
     * \brief Converts a cv::Mat to a QImage.
     * \param mat cv::Mat to be converted.
     * \return The resulting QImage.
     */
    QImage mat2qimage(cv::Mat& mat);

    /*!
     * \brief Converts the integer description of an image type to string.
     * \param type The integer description of an image.
     * \return The resulting string.
     */
    std::string type2str(int type);

    /*!
     * \brief Populate a pose service object with the wanted robotic manipulator pose.
     * \param relative True if the position is relative to current pose. False if the position is absolute in relation to the world reference frame.
     * \param position Include position in the pose. If false, the manipulator will only reorient.
     * \param x Desired position in X direction.
     * \param y Desired position in Y direction.
     * \param z Desired position in Z direction.
     * \param orientation Include orientation in the pose. If false, the manipulator will only reposition.
     * \param roll Desired roll angle.
     * \param pitch Desired pitch angle.
     * \param yaw Desired yaw angle.
     */
    void setPoseRequest(bool relative, bool position, double x, double y, double z, bool orientation, double roll, double pitch, double yaw);

    /*!
     * \brief Return the detected objects position in the x axis, in image coordinate.
     * \return The detected objects x position in image coordinates.
     */
    double getXoffset();

    /*!
     * \brief Returns the detected objects position in the y axis, in image coordinates.
     * \return The detected objects y position in image coordinates.
     */
    double getYoffset();

    /*!
     * \brief Returns the current z-depth scaling factor.
     * \return The current z-depth scaling factor.
     */
    double getTheta();

    /*!
     * \brief Returns the value of the counter.
     * \return Current value of the counter.
     */
    int getCounter();

    /*!
     * \brief Returns the current pose of the Agilus 1 robotic manipulator.
     * \return The current pose of Agilus 1.
     */
    geometry_msgs::PoseStamped getAg1CurrentPose();

    /*!
     * \brief Returns the current pose of the Agilus 2 robotic manipulator.
     * \return The current pose of Agilus 2.
     */
    geometry_msgs::PoseStamped getAg2CurrentPose();

public Q_SLOTS:
    //Slots used to recieve events from one another class. All slots and signals are connected in main_window.cpp

    /*!
     * \brief Subscribes to a 3D point cloud topic.
     * \param topic The 3D point cloud topic name.
     */
    void subscribeToPointCloud2(QString topic);

    /*!
     * \brief Subscribes to a 2D video topic.
     * \param topic The 2D video feed topic name.
     */
    void subscribeTo2DobjectDetected(QString topic);

    /*!
     * \brief Sets the runnig status of the image processor.
     * \param processRunning The new running status.
     */
    void setProcessImageRunning(bool processRunning);

    /*!
     * \brief Turns on/off image color in the image processor.
     * \param color Color on/off.
     */
    void setProcessImageColor(bool color);

    /*!
     * \brief Turns on/off distortion correction in the image procesor.
     * \param undistort Distortion correction on/off.
     */
    void setProcessImageUndistort(bool undistort);

    /*!
     * \brief Switches between bruteforce and FLANN matching in the image processor.
     * \param bruteforce True = bruteforce, False = FLANN.
     */
    void setProcessImageBruteforce(bool bruteforce);

    /*!
     * \brief Sets the keypoint selector and descriptor estimator in the image processor.
     * \param keypoint Selected keypoint selector.
     * \param descriptor Selected descriptor estimator.
     */
    void setProcessImageKeypointDescriptor(std::string keypoint, std::string descriptor);

    /*!
     * \brief Sets the file path to the image used for 2D object detection in the image processor.
     * \param imagePath The file path to the image used for matching in the image processor.
     */
    void setProcessImageMatchingPicture(std::string imagePath);

    /*!
     * \brief Sets the z-depth scaling factor used to convert from pixel to image coordinates in the image processor.
     * \param lambda The z-depth scaling factor.
     */
    void setProcessImageDepthLambda(double lambda);

    /*!
     * \brief Initiates a linear movement for agilus 1.
     * \param x Position in X direction for the new manipulator pose.
     * \param y Position in Y direction for the new manipulator pose.
     * \param z Position in Z direction for the new manipulator pose.
     * \param roll Roll angle for the new manipulator pose.
     * \param pitch Pitch angle for the new manipulator pose.
     * \param yaw Yaw angle for the new manipulator pose.
     */
    void move_ag1(double x, double y, double z, double roll, double pitch, double yaw);

    /*!
     * \brief Initiates a linear movement for agilus 2.
     * \param x Position in X direction for the new manipulator pose.
     * \param y Position in Y direction for the new manipulator pose.
     * \param z Position in Z direction for the new manipualtor pose.
     * \param roll Roll angle for the new manipulator pose.
     * \param pitch Pitch angle for the new manipulator pose.
     * \param yaw Yaw angle for the new manipulator pose.
     */
    void move_ag2(double x, double y, double z, double roll, double pitch, double yaw);

    /*!
     * \brief Plans a linear movement for agilus 1.
     * \param x Position in X direction for the planned manipualtor pose.
     * \param y Position in Y direction for the planned manipulator pose.
     * \param z Position in Z direction for the planned manipulator pose.
     * \param roll Roll angle for the planned manipulator pose.
     * \param pitch Pitch angle for the planned manipulator pose.
     * \param yaw Yaw angle for the planned manipulator pose.
     */
    void plan_ag1(double x, double y, double z, double roll, double pitch, double yaw);

    /*!
     * \brief Plans a linear movement for agilus 2.
     * \param x Position in X direction for the planned manipulator pose.
     * \param y Position in Y direction for the planned manipulator pose.
     * \param z Position in Z direction for the planned manipulator pose.
     * \param roll Roll angle for the planned manipulator pose.
     * \param pitch Pitch angle for the planned manipulator pose.
     * \param yaw Yaw angle for the planned manipulator pose.
     */
    void plan_ag2(double x, double y, double z, double roll, double pitch, double yaw);

    /*!
     * \brief Initiates a "Close gripper" action for agilus 1.
     */
    void closeAG1Gripper();

    /*!
     * \brief Initiates a "Open gripper" action for agilus 1.
     */
    void openAG1Gripper();


Q_SIGNALS:
    //Signals used to emit event from one class to another. All signals are connected in main_window.cpp

    /*!
     * \brief Signals the user interface to close upon a ros shutdown.
     */
    void rosShutdown();

    /*!
     * \brief Signals the user interface to update the visualized point cloud.
     * \param cloud The new 3D point cloud.
     */
    void updatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /*!
     * \brief Signals the user interface to update the visualized 2D image
     * \param image The new 2D image.
     */
    void update2Dimage(QImage image);

    /*!
     * \brief Signals the user interface in the case where the robotic manipulators are unreachable.
     */
    void disableAuto();

private:
    int init_argc; //!< Initial condition.
    char** init_argv; //!< Initial condition.
    QStringListModel logging_model; //!< String list model used to report data back to the used.
    ros::Subscriber pointCloud2Sub; //!< Subscriber used for acquiring 3D point clouds.
    ros::Subscriber object2DdetectedSub; //!< Subscriber used for acquiring video feed from the image processor.
    ros::Subscriber object2Dpose; //!< Subscriber used to acquire the pose of the detected object from image processor.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; //!< 3D point cloud.
    boost::atomic_bool imageReading; //!< Flag used when reading video feed.

    // Image processor
    ros::ServiceClient getImageprocessorRunning; //!< Service client used to get the running status of the image processor.
    ros::ServiceClient setImageprocessorRunning; //!< Service client used to set the running status of the image processor.
    ros::ServiceClient getImageprocessorColor; //!< Service client used to get the color status of the image processor.
    ros::ServiceClient setImageprocessorColor; //!< Service client used to set the color status of the image processor.
    ros::ServiceClient getImageprocessorBruteforce; //!< Service client used to get the matching status of the image processor.
    ros::ServiceClient setImageprocessorBruteforce; //!< Service client used to toggle between bruteforce and FLANN matching in the image processor.
    ros::ServiceClient getImageprocessorUndistort; //!< Service client used to get the distortion correction status of the image processor.
    ros::ServiceClient setImageprocessorUndistort; //!< Service client used to set the distortion correction status of the image processor.
    ros::ServiceClient getImageprocessorKeypoint; //!< Service client used to get the keypoint selector used in the image processor.
    ros::ServiceClient setImageprocessorKeypoint; //!< Service client used to set the keypoint selector used in the image processor.
    ros::ServiceClient getImageprocessorDescriptor; //!< Service client used to get the descriptor type used in the image processor.
    ros::ServiceClient setImageprocessorDescriptor; //!< Service client used to set the descriptor type used in the image processor.
    ros::ServiceClient setImageprocessorMatchingImage; //!< Service client used to set the image used for matching in the image processor.
    ros::ServiceClient setImageprocessorDepth; //!< Service client used to set z-distance scaling factor for the 2D object detection.

    image_processor::getProcessRunning getImproRunning; //!< Service object used to call the getProcessRunning service.
    image_processor::setProcessRunning setImproRunning; //!< Service object used to call the setProcessRunning service.
    image_processor::getVideoColor getVideoColor; //!< Service object used to call the getVideoColor service.
    image_processor::setVideoColor setVideoColor; //!< Service object used to call the setVideoColor service.
    image_processor::getVideoUndistortion getVideoUndist; //!< Service object used to call the getVideoUndistortion service.
    image_processor::setVideoUndistortion setVideoUndist; //!< Service object used to call the setVideoUndistortion service.
    image_processor::getBruteforceMatching getBF; //!< Service object used to call the getBruteforceMatching service.
    image_processor::setBruteforceMatching setBF; //!< Service object used to call the setBruteforceMatching service.
    image_processor::getKeypointDetectorType getKeypoint; //!< Service object used to call the getKeypointDetectorType service.
    image_processor::setKeypointDetectorType setKeypoint; //!< Service object used to call the setKeypointDetectorType service.
    image_processor::getDescriptorType getDescriptor; //!< Service object used to call the getDescriptorType service.
    image_processor::setDescriptorType setDescriptor; //!< Service object used to call the setDescriptorType service.
    image_processor::setMatchingImage1 setMatchingImage; //!< Service object used to call the setMatchingImage1 service.
    image_processor::setImageDepth setImageDepth; //!< Service object used to call the setImageDepth service.

    // Robot control
    ros::ServiceClient goToClient_ag1; //!< Service client used to request movement for agilus 1.
    ros::ServiceClient goToClient_ag2; //!< Service client used to request movement for agilus 2.
    ros::ServiceClient planClient_ag1; //!< Service client used to plan movement for agilus 1.
    ros::ServiceClient planClient_ag2; //!< Service client used to plan movement for agilus 2.
    ros::ServiceClient setAgilus1Digout; //!< Service client used to control digital output on agilus 1.
    agilus_planner::Pose pose_service; //!< Service object used for robotic motion.
    kuka_rsi_hw_interface::write_8_outputs agilusDigout; //!< Service object used to control digital output on agilus 1.

    moveit::planning_interface::MoveGroup *ag1; //!< Move group used to acquire manipulator pose for agilus 1.
    moveit::planning_interface::MoveGroup *ag2; //!< Move group used to acquire manipulator pose for agilus 2.

    double x_object, y_object, theta_object; //!< Image coordinate for the 2D detected object.
    int counter; //!< Variable used to ensure reliable data from 2D image data stream.
};
}  // namespace agilus_master_project

#endif

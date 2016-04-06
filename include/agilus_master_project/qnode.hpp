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

public Q_SLOTS:
    void subscribeToPointCloud2(QString topic);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void updatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;
    ros::Subscriber pointCloud2Sub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

}  // namespace agilus_master_project

#endif /* agilus_master_project_QNODE_HPP_ */

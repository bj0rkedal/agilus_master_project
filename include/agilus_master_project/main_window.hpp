/**
 * @file /include/agilus_master_project/main_window.hpp
 *
 * @brief Qt based gui for agilus_master_project.
 *
 * @date November 2010
 **/
#ifndef agilus_master_project_MAIN_WINDOW_H
#define agilus_master_project_MAIN_WINDOW_H


#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "modelloader.hpp"
#include "pcl_filters.hpp"
#include <vtkRenderWindow.h>
#include <QVTKWidget.h>


namespace agilus_master_project {


/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function
    void displayNewPointCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
    void drawShapes();


public Q_SLOTS:
    //Autoconnected
     void on_loadPCDButton_clicked(bool check);
     void on_subscribeToTopicButton_clicked(bool check);
     void on_detectObjectsButton_clicked(bool check);
     void updatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
     void printToLog(QString text);

Q_SIGNALS:
     void subscribeToPointCloud2(QString topic);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    QVTKWidget *w1;
    std::vector<pcl::visualization::Camera> cam;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentCloud;
    PclFilters *filters;
    ModelLoader *box;
    ModelLoader *cone;
    ModelLoader *freak;
    std::vector<ModelLoader*> models;
    QStringList objects;
    bool runStream;
    Eigen::Affine3f frameA,frameB,tag,worldFrame,camera;
    Eigen::Matrix4f cameraToTag, world, tagToCamera, worldToTag, partAInTag, partBInTag, tempWorld, tagToWorld;
};

}  // namespace agilus_master_project

#endif // agilus_master_project_MAIN_WINDOW_H

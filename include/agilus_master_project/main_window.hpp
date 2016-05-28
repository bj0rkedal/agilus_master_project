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
#include "opencv2/core.hpp"
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>


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
    void init();
    void init_descriptor_keypoint_combobox();
    void initRobotUI();
    void detect3D(int partAIndex, int partBIndex);
    void detectAngle2D(double &part);


public Q_SLOTS:
    //Autoconnected
     void on_loadPCDButton_clicked(bool check);
     void on_subscribeToTopicButton_clicked(bool check);
     void on_subscribeToTopicButton2_clicked(bool check);
     void on_detectObjectsButton_clicked(bool check);
     void updatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
     void update2Dimage(QImage image);
     void printToLog(QString text);
     void on_saveImageButton_clicked(bool check);
     void on_boxesCheckbox_clicked(bool check);
     void on_framesCheckbox_clicked(bool check);
     void on_runningCheckBox_clicked(bool check);
     void on_colorCheckBox_clicked(bool check);
     void on_undistortCheckBox_clicked(bool check);
     void on_bruteforceCheckBox_clicked(bool check);
     void on_updateSettingsPushButton_clicked(bool check);
     void on_imageToDetectButton_clicked(bool check);
     void on_planTrajPushButton_clicked(bool check);
     void on_moveManipPushButton_clicked(bool check);
     void on_homeManipPushButton_clicked(bool check);
     void on_testPlanButton_clicked(bool check);
     void on_testMoveButton_clicked(bool check);
     void on_plan2DcorrButton_clicked(bool check);
     void on_move2DcorrButton_clicked(bool check);
     void on_setDepthPushButton_clicked(bool check);
     void on_openGripperButton_clicked(bool check);
     void on_closeGripperButton_clicked(bool check);
     void on_autoDetection3dButton_clicked(bool check);
     void on_auto2dFirstPartButton_clicked(bool check);
     void on_auto2dFirstPartAngleButton_clicked(bool check);
     void on_auto2dSecondPartButton_clicked(bool check);
     void on_auto2dSecondPartAngleButton_clicked(bool check);
     void on_moveGripperPartAButton_clicked(bool check);
     void on_moveGripperPartBButton_clicked(bool check);
     void on_assemblePartsButton_clicked(bool check);
     void on_worldCoordinatesCheckBox_clicked(bool check);
     void on_robotComboBox_currentIndexChanged(int i);
     void on_resetSequenceButton_clicked(bool check);
     void on_create_training_set_button_clicked(bool check);
     void disableAuto();

Q_SIGNALS:
     void subscribeToPointCloud2(QString topic);
     void subscribeTo2DobjectDetected(QString topic);
     void setProcessImageRunning(bool processRunning);
     void setProcessImageColor(bool color);
     void setProcessImageUndistort(bool undistort);
     void setProcessImageBruteforce(bool bruteforce);
     void setProcessImageKeypointDescriptor(std::string keypoint, std::string descriptor);
     void setProcessImageMatchingPicture(std::string imagePath);
     void setProcessImageDepthLambda(double lambda);
     void move_ag1(double x, double y, double z,
                   double roll, double pitch, double yaw);
     void move_ag2(double x, double y, double z,
                   double roll, double pitch, double yaw);
     void plan_ag1(double x, double y, double z,
                   double roll, double pitch, double yaw);
     void plan_ag2(double x, double y, double z,
                   double roll, double pitch, double yaw);
     void closeAG1Gripper();
     void openAG1Gripper();

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
    bool movedToPartA, movedToPartB;
    QImage saveImage;
    Eigen::Affine3f frameA,frameB,tag,worldFrame,camera;
    Eigen::Matrix4f cameraToTag, world, tagToCamera, worldToTag, partAInTag, partBInTag, tempWorld, tagToWorld;
    double homeX, homeY1, homeY2, homeZ, homeRoll, homePitch, homeYaw;
    double anglePartA, anglePartB;

    std::string partApath;
    std::string partBpath;
};

}  // namespace agilus_master_project

#endif // agilus_master_project_MAIN_WINDOW_H

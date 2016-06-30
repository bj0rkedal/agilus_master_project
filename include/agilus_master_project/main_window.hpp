//
// Original author Asgeir Bjoerkedal & Kristoffer Larsen. Latest change date 01.05.2016
// main_window.cpp is responsible for all user interaction and information flow. Because of this, the majority
// of the methods implemented are used for user interaction.
//
// Created as part of the software solution for a Master's Thesis in Production Technology at NTNU Trondheim.
//

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
 * @brief Entry point for the user interaction.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
    /*!
     * \brief Constructor for the MainWindow class.
     * \param argc Initial argument.
     * \param argv Initial argiment.
     * \param parent Parent argument, default = 0.
     */
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

    /*!
     * \brief Event called when the user interface window is closed.
     * \param event Closing event.
     */
	void closeEvent(QCloseEvent *event); // Overloaded function

    /*!
     * \brief Displays a new PCL visualizer in the user interface.
     * \param vis The new PCL visualizer to display.
     */
    void displayNewPointCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

    /*!
     * \brief Initializing method used to ready all local settings and parameters.
     */
    void drawShapes();

    /*!
     * \brief Initializing method used to populate all lists and comboboxes, as well as set the stat of all buttons.
     */
    void init();

    /*!
     * \brief Initializing method used to populate all graphical elements related to 2D obect detection.
     */
    void init_descriptor_keypoint_combobox();

    /*!
     * \brief Initializing method used to populate all graphical elements related to robotic manipualtor control.
     */
    void initRobotUI();

    /*!
     * \brief Performs a 3D object detection routine based on user set models.
     * \param partAIndex Index for the model used for part A in the user selectable combobox.
     * \param partBIndex Index for the model used for part B in the use selectable combobox.
     */
    void detect3D(int partAIndex, int partBIndex);

    /*!
     * \brief Records 20 samples of current part angle (detected using 2D object detection) and calcualtes an average value.
     * \param part Resulting angle.
     */
    void detectAngle2D(double &part);


public Q_SLOTS:
     //Slots used to recieve events from one another class. All slots and signals are connected in main_window.cpp

    /*!
      * \brief Event for the "Load PCD" button. Loads and displays a selected .PCD file in the user interface.
      * \param check Button status for the event.
      */
     void on_loadPCDButton_clicked(bool check);

     /*!
      * \brief Event for the "Subscribe to topic" button. Subscribes to the selected 3D point cloud topic.
      * \param check Button status for the event.
      */
     void on_subscribeToTopicButton_clicked(bool check);

     /*!
      * \brief Event for the "Subscribe to topic" button. Subscribes to the selected 2D video feed topic.
      * \param check Button status for the event.
      */
     void on_subscribeToTopicButton2_clicked(bool check);

     /*!
      * \brief Event for the "Detect object" button. Attempts a 3D object detection process based on user selected models.
      * \param check Button status for the event.
      */
     void on_detectObjectsButton_clicked(bool check);

     /*!
      * \brief Updates the currently displayed point cloud.
      * \param cloud The new point cloud that is to be displayed.
      */
     void updatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

     /*!
      * \brief Updates the currently displayed 2D image.
      * \param image The new image that is to be displayed.
      */
     void update2Dimage(QImage image);

     /*!
      * \brief Prints a specified text in the user interface. This is used to give feedback to the user.
      * \param text Text to print to log.
      */
     void printToLog(QString text);

     /*!
      * \brief Event for the "Save image" button. The currently displayed 2D image is saved to the user specified path.
      * \param check Button status for the event.
      */
     void on_saveImageButton_clicked(bool check);

     /*!
      * \brief Event for the "Boxes" checkbox. This toggles the illustration of the boxes in the 3D visualizer. The boxes illustrates
      *        area in which the two parts is searched for during the 3D object detection process.
      * \param check Click status of the event.
      */
     void on_boxesCheckbox_clicked(bool check);

     /*!
      * \brief Event for the "Frames" checkbox. This toggles the illustration of the different reference frames in the 3D visualizer.
      * \param check Click status of the event.
      */
     void on_framesCheckbox_clicked(bool check);

     /*!
      * \brief Event for the "Running" checkbox. This toggles the 2D object detection on/off.
      * \param check Click status of the event.
      */
     void on_runningCheckBox_clicked(bool check);

     /*!
      * \brief Event for the "Color" checkbox. This toggles between color and grayscale 2D video feed from the image processor.
      * \param check Click status of the event.
      */
     void on_colorCheckBox_clicked(bool check);

     /*!
      * \brief Event for the "Undistort" checkbox. This toggles the distortion correction on/off.
      * \param check Click status of the event.
      */
     void on_undistortCheckBox_clicked(bool check);

     /*!
      * \brief Event for the "Bruteforce" checkbox. This toggle between bruteforce and FLANN 2D matching.
      * \param check Click status of the event.
      */
     void on_bruteforceCheckBox_clicked(bool check);

     /*!
      * \brief Event for th "Update settings" button. This sends the current configuration to the image processor.
      * \param check Button status for the event.
      */
     void on_updateSettingsPushButton_clicked(bool check);

     /*!
      * \brief Event for the "Image to detect" button. This updates the image to match aginst in the image processor.
      * \param check Button status for the event.
      */
     void on_imageToDetectButton_clicked(bool check);

     /*!
      * \brief Event for the "Plan trajectory" button. This sends a plan move request to QNode.
      * \param check Button status for the event.
      */
     void on_planTrajPushButton_clicked(bool check);

     /*!
      * \brief Event for the "Move trajectory" button. This sends a initiate movement request to QNode.
      * \param check Button status for the event.
      */
     void on_moveManipPushButton_clicked(bool check);

     /*!
      * \brief Event for the "Home manipulator" button. This sends a initiate movement to homo position request to QNode.
      * \param check Button status for the event.
      */
     void on_homeManipPushButton_clicked(bool check);

     /*!
      * \brief Event for the "Plane 2D correction" button. This sends a plan move request to QNode based on the position
      *        of the part in the 2D camera view.
      * \param check Button status for the event.
      */
     void on_plan2DcorrButton_clicked(bool check);

     /*!
      * \brief Event for the "Move 2D correction" button. This corrects the position of agilus 2 based on the position
      *        of the part in the 2D camera view.
      * \param check Button status for the event.
      */
     void on_move2DcorrButton_clicked(bool check);

     /*!
      * \brief Event for the "Set depth" button. This sends the used specified value for the z-depth scaling factor to
      *        the image processor.
      * \param check Button status for the event.
      */
     void on_setDepthPushButton_clicked(bool check);

     /*!
      * \brief Event for the "Open gripper" button. This sends a "Open gripper" request to QNode.
      * \param check Button status for the event.
      */
     void on_openGripperButton_clicked(bool check);

     /*!
      * \brief Event for the "Close gripper" button. This sends a "Close gripper" request to QNode.
      * \param check Button status for the event.
      */
     void on_closeGripperButton_clicked(bool check);

     /*!
      * \brief Event for the "Auto 3D object detection" button. This performs a 3D object detection with preconfigured
      *        parameters.
      * \param check Button status for the event.
      */
     void on_autoDetection3dButton_clicked(bool check);

     /*!
      * \brief Event for the "Auto 2D first part" button. This records the current position of the detected object (using 2D detection).
      * \param check Button status for the event.
      */
     void on_auto2dFirstPartButton_clicked(bool check);

     /*!
      * \brief Event for the "Auto 2D first part" button. This records the current orientation of the detected object (using 2D detection).
      * \param check Button status for the event.
      */
     void on_auto2dFirstPartAngleButton_clicked(bool check);

     /*!
      * \brief Event for the "Auto 2D second part" button. This records the current position of the detected object (using 2D detection).
      * \param check Button status for the event.
      */
     void on_auto2dSecondPartButton_clicked(bool check);

     /*!
      * \brief Event for the "Auto 2D second part" button. This records the current orientation of the detected object (using 2D detection).
      * \param check Button status for the event.
      */
     void on_auto2dSecondPartAngleButton_clicked(bool check);

     /*!
      * \brief Event for the "Move gripper to part A" button. This moves agilus 1 to the position of the object detected in the 2D image (part A).
      * \param check Button status for the event.
      */
     void on_moveGripperPartAButton_clicked(bool check);

     /*!
      * \brief Event for the "Move gripper to part B" button. This moves agilus 1 to the position of the object detected in the 2D image (part B).
      * \param check Button status for the event.
      */
     void on_moveGripperPartBButton_clicked(bool check);

     /*!
      * \brief Event for the "Assemble" button. This initiate the robotic movement needed to assemble the two parts.
      * \param check Button status for the event.
      */
     void on_assemblePartsButton_clicked(bool check);

     /*!
      * \brief Event for the "World coordinates" checkbox. This toggles between relative and aboslute robotic movements.
      * \param check Click status for the event.
      */
     void on_worldCoordinatesCheckBox_clicked(bool check);

     /*!
      * \brief Event for the "Selected robot" combobox. This is called when the selected robot is changed.
      * \param i The selected index from the combobox. Agilus 1 = 1, Agilus 2 = 2.
      */
     void on_robotComboBox_currentIndexChanged(int i);

     void on_objectsListA_currentIndexChanged(int i);

     void on_objectsListB_currentIndexChanged(int i);

     /*!
      * \brief Event for the "Reset sequence" button. This resets all the data acquired through the previous automated assembly and
      *        readys the program to run a new autmated assembly.
      * \param check Button status for the event.
      */
     void on_resetSequenceButton_clicked(bool check);

     /*!
      * \brief Event for "create training set" button. Creates a training set based on the user input.
      * \param check Button status for the event.
      */
     void on_create_training_set_button_clicked(bool check);

     /*!
      * \brief Disables all robotic manipulator control.
      */
     void disableAuto();

Q_SIGNALS:
     //Signals used to emit events from one class to another. All signals are connected in main_window.cpp

     /*!
      * \brief Signals QNode to subscribe to a 3D point cloud data feed topic.
      * \param topic The topic name.
      */
     void subscribeToPointCloud2(QString topic);

     /*!
      * \brief Signals QNode to subscirbe to a 2D video feed topic.
      * \param topic The topic name.
      */
     void subscribeTo2DobjectDetected(QString topic);

     /*!
      * \brief Signals QNode to set the running status of the image processor.
      * \param processRunning Wanted running status of the image processor.
      */
     void setProcessImageRunning(bool processRunning);

     /*!
      * \brief Signals QNode to turn on/off image color.
      * \param color True for color, False for grayscale.
      */
     void setProcessImageColor(bool color);

     /*!
      * \brief Signals QNode to turn on/off image distortion correction
      * \param undistort True for distortion correction, False for no correction.
      */
     void setProcessImageUndistort(bool undistort);

     /*!
      * \brief Signals QNode to selecte matching method.
      * \param bruteforce True for bruteforce matching, False for FLANN matching.
      */
     void setProcessImageBruteforce(bool bruteforce);

     /*!
      * \brief Signals QNode to set keypoint selector and descriptor estimator.
      * \param keypoint Selected keypoint selector.
      * \param descriptor Selected descriptor estimator.
      */
     void setProcessImageKeypointDescriptor(std::string keypoint, std::string descriptor);

     /*!
      * \brief Signals QNode to set the file path for the matching image used in 2D object detection.
      * \param imagePath The file path to the image used for matching.
      */
     void setProcessImageMatchingPicture(std::string imagePath);

     /*!
      * \brief Signals QNode to set the z-depth scaling factor.
      * \param lambda The z-depth scaling factor.
      */
     void setProcessImageDepthLambda(double lambda);

     /*!
      * \brief Signals QNode to initiate a linear movement for agilus 1.
     * \param x Position in X direction for the new manipulator pose.
     * \param y Position in Y direction for the new manipulator pose.
     * \param z Position in Z direction for the new manipulator pose.
     * \param roll Roll angle for the new manipulator pose.
     * \param pitch Pitch angle for the new manipulator pose.
     * \param yaw Yaw angle for the new manipulator pose.
      */
     void move_ag1(double x, double y, double z, double roll, double pitch, double yaw);

     /*!
      * \brief Signals QNode to initiate a linear movement for agilus 2.
     * \param x Position in X direction for the new manipulator pose.
     * \param y Position in Y direction for the new manipulator pose.
     * \param z Position in Z direction for the new manipualtor pose.
     * \param roll Roll angle for the new manipulator pose.
     * \param pitch Pitch angle for the new manipulator pose.
     * \param yaw Yaw angle for the new manipulator pose.
      */
     void move_ag2(double x, double y, double z, double roll, double pitch, double yaw);

     /*!
      * \brief Signals QNode to plan a linear movement for agilus 1.
     * \param x Position in X direction for the planned manipualtor pose.
     * \param y Position in Y direction for the planned manipulator pose.
     * \param z Position in Z direction for the planned manipulator pose.
     * \param roll Roll angle for the planned manipulator pose.
     * \param pitch Pitch angle for the planned manipulator pose.
     * \param yaw Yaw angle for the planned manipulator pose.
      */
     void plan_ag1(double x, double y, double z, double roll, double pitch, double yaw);

     /*!
      * \brief Signals QNode to plan a linear movement for agilus 2.
     * \param x Position in X direction for the planned manipulator pose.
     * \param y Position in Y direction for the planned manipulator pose.
     * \param z Position in Z direction for the planned manipulator pose.
     * \param roll Roll angle for the planned manipulator pose.
     * \param pitch Pitch angle for the planned manipulator pose.
     * \param yaw Yaw angle for the planned manipulator pose.
      */
     void plan_ag2(double x, double y, double z, double roll, double pitch, double yaw);

     /*!
      * \brief Signals QNode to perform a "Close gripper" action on agilus 1.
      */
     void closeAG1Gripper();

     /*!
      * \brief Signals QNode to perform a "Open gripper" action on agilus 1.
      */
     void openAG1Gripper();

private:
    Ui::MainWindowDesign ui; //!< Object of the user interface defention XML (this XML defines the graphical apperance of the application).
    QNode qnode; //!< Local object of the QNode class.
    QVTKWidget *w1; //!< QVTK Widget used to display the PCL visualizer contained in the user interface.
    std::vector<pcl::visualization::Camera> cam; //!< Vector containing the current camera parameters used in the PCL visualizer.
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1; //!< PCL visualizer used to visualize 3D point clouds.
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentCloud; //!< 3D point cloud.
    PclFilters *filters; //!< Local object of the PclFilters class.
    ModelLoader *box; //!< Modelloader for a test part (a box).
    ModelLoader *cone; //!< Modelloader for part B.
    ModelLoader *freak; //!< Modelloader for part A.
    std::vector<ModelLoader*> models; //!< Vector containing all modelloaders used to load the training sets.
    QStringList objects; //!< List containing all the loaded training sets.
    bool runStream; //!< Flag used to start and stop the 3D point cloud data stream.
    bool movedToPartA, movedToPartB; //!< Flag used when performing automated assembly.
    QImage saveImage; //!< Copy of the 2D image that is to be saved to file.
    Eigen::Affine3f frameA,frameB,tag,worldFrame,camera; //!< Predefined reference frames for part A, part B, tag, world and 3D camera.
    Eigen::Matrix4f cameraToTag, world, tagToCamera, worldToTag, partAInTag, partBInTag, tempWorld, tagToWorld; //!< Homogenous transformation matrices used to transform the object position.
    double homeX, homeY1, homeY2, homeZ, homeRoll, homePitch, homeYaw; //!< Predefined values for the home position of the robotic manipulators.
    double anglePartA, anglePartB; //!< Recorded angle for part A and part B.
    double pickUpHeight, deployHeight; //!< Height used for picking up and deploying different parts.
    double detectionHeightA, detectionHeightB; //!< Height used when detecting objects in 2D

    std::string partApath; //!< Path to the matching image used for part A.
    std::string partBpath; //!< Path to the matching image used for part B.
};

}  // namespace agilus_master_project

#endif // agilus_master_project_MAIN_WINDOW_H

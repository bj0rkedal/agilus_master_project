/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/


#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/agilus_master_project/main_window.hpp"



namespace agilus_master_project {

using namespace Qt;


MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
    , currentCloud(new pcl::PointCloud<pcl::PointXYZ>)
{
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZ>::Ptr >("pcl::PointCloud<pcl::PointXYZ>::Ptr");
    qRegisterMetaType<cv::Mat>("cv::Mat");
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(this,SIGNAL(subscribeToPointCloud2(QString)),&qnode,SLOT(subscribeToPointCloud2(QString)));
    QObject::connect(this,SIGNAL(subscribeTo2DobjectDetected(QString)),&qnode,SLOT(subscribeTo2DobjectDetected(QString)));
    QObject::connect(&qnode,SIGNAL(updatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr)),this,SLOT(updatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr)));
    QObject::connect(&qnode,SIGNAL(update2Dimage(QImage)),this,SLOT(update2Dimage(QImage)));
    QObject::connect(this,SIGNAL(setProcessImageRunning(bool)),&qnode,SLOT(setProcessImageRunning(bool)));
    QObject::connect(this,SIGNAL(setProcessImageColor(bool)),&qnode,SLOT(setProcessImageColor(bool)));
    QObject::connect(this,SIGNAL(setProcessImageUndistort(bool)),&qnode,SLOT(setProcessImageUndistort(bool)));
    QObject::connect(this,SIGNAL(setProcessImageBruteforce(bool)),&qnode,SLOT(setProcessImageBruteforce(bool)));
    QObject::connect(this,SIGNAL(setProcessImageKeypointDescriptor(std::string,std::string)),&qnode,SLOT(setProcessImageKeypointDescriptor(std::string,std::string)));
    QObject::connect(this,SIGNAL(setProcessImageMatchingPicture(std::string)),&qnode,SLOT(setProcessImageMatchingPicture(std::string)));
    QObject::connect(this,SIGNAL(plan_ag1(double,double,double,double,double,double)),&qnode,SLOT(plan_ag1(double,double,double,double,double,double)));
    QObject::connect(this,SIGNAL(plan_ag2(double,double,double,double,double,double)),&qnode,SLOT(plan_ag2(double,double,double,double,double,double)));
    QObject::connect(this,SIGNAL(move_ag1(double,double,double,double,double,double)),&qnode,SLOT(move_ag1(double,double,double,double,double,double)));
    QObject::connect(this,SIGNAL(move_ag2(double,double,double,double,double,double)),&qnode,SLOT(move_ag2(double,double,double,double,double,double)));
    QObject::connect(this,SIGNAL(setProcessImageDepthLambda(double)),&qnode,SLOT(setProcessImageDepthLambda(double)));
    QObject::connect(this,SIGNAL(closeAG1Gripper()),&qnode,SLOT(closeAG1Gripper()));
    QObject::connect(this,SIGNAL(openAG1Gripper()),&qnode,SLOT(openAG1Gripper()));
    init();
    init_descriptor_keypoint_combobox();
    initRobotUI();
}

MainWindow::~MainWindow() {}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

void MainWindow::displayNewPointCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
{
    runStream = false;
    viewer1->getCameras(cam);
    viewer1 = vis;
    w1->SetRenderWindow(viewer1->getRenderWindow());
    viewer1->setCameraPosition(cam[0].pos[0],cam[0].pos[1],cam[0].pos[2],cam[0].view[0],cam[0].view[1],cam[0].view[2]);
    drawShapes();
}

void MainWindow::drawShapes()
{
    viewer1->removeAllShapes();
    if(ui.boxesCheckbox->isChecked()){
        viewer1->addCube(-0.510, -0.130, -0.5, 0.3, 0.76, 1.19, 0, 1.0, 0, "partA", 0);
        viewer1->addCube(-0.130, 0.27, -0.5, 0.3, 0.76, 1.19, 1.0, 0, 0, "partB", 0);
    }
    if(ui.framesCheckbox->isChecked()){
        viewer1->addCoordinateSystem(0.2,tag);
        viewer1->addCoordinateSystem(0.2,camera);
        viewer1->addCoordinateSystem(0.5,worldFrame);
    }
    w1->update();
}

void MainWindow::init()
{
    ui.outputTabManager->setCurrentIndex(0);
    ui.settingsTabManager->setCurrentIndex(0);
    filters = new PclFilters();
    runStream = false;

    box = new ModelLoader("nuc-42-100");
    box->populateLoader();
    cone = new ModelLoader("cone-42-200");
    cone->populateLoader();
    freak = new ModelLoader("freakthing-42-200");
    freak->populateLoader();
    models.push_back(box);
    models.push_back(cone);
    models.push_back(freak);

    w1 = new QVTKWidget();
    viewer1.reset(new pcl::visualization::PCLVisualizer ("3D Viewer", false));
    w1->SetRenderWindow(viewer1->getRenderWindow());
    viewer1->setCameraPosition( 0.146598, 0.0941454, -4.95334, -0.0857047, -0.0396425, 0.600109, -0.00146821, -0.999707, -0.0241453, 0);
    ui.pointCloudLayout->addWidget(w1);

    cameraToTag << -0.000762713, -0.999832,   -0.0182927, -0.116073,
            -0.569249,     0.0154738,  -0.822019,  -0.0416634,
            0.822165,     0.00978617, -0.569166,   1.0405,
            0,            0,           0,          1;
    tag = cameraToTag;
    world << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    worldToTag << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0.87,
                0, 0, 0, 1;

    tagToWorld = worldToTag.inverse();
    tagToCamera = cameraToTag.inverse();
    tempWorld = cameraToTag*tagToWorld;
    worldFrame = tempWorld;
    camera = world;

    drawShapes();

    qnode.init();
    objects.append("nuc box");
    objects.append("cone");
    objects.append("freak thing");
    ui.objectsListA->addItems(objects);
    ui.objectsListB->addItems(objects);

    ui.testPlanButton->setEnabled(false);
    ui.testMoveButton->setEnabled(false);


    QStringList list, list2;
    QString tmp;
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
        const ros::master::TopicInfo& info = *it;
        //std::cout << "Topic : " << it - master_topics.begin() << ": " << info.name << " -> " << info.datatype <<       std::endl;
        tmp = QString::fromUtf8(info.datatype.c_str());

        // Add more types if needed
        if(QString::compare(tmp, "sensor_msgs/PointCloud2", Qt::CaseInsensitive) == 0){
            list.append(QString::fromUtf8(info.name.c_str()));
        }
    }
    ui.topicComboBox->addItems(list);

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
        const ros::master::TopicInfo& info = *it;
        //std::cout << "Topic : " << it - master_topics.begin() << ": " << info.name << " -> " << info.datatype <<       std::endl;
        tmp = QString::fromUtf8(info.datatype.c_str());

        // Add more types if needed
        if(QString::compare(tmp, "sensor_msgs/Image", Qt::CaseInsensitive) == 0){
            QString tmp = QString::fromUtf8(info.name.c_str());
            if(tmp.endsWith("image_color")){
                list2.append(QString::fromUtf8(info.name.c_str()));
            } else if(tmp.endsWith("image")){
                list2.append(QString::fromUtf8(info.name.c_str()));
            }
        }
    }
    ui.topicComboBox2->addItems(list2);
    ui.logViewer->insertPlainText("User interface initiated");
}

void MainWindow::init_descriptor_keypoint_combobox()
{
    QStringList keyPointLabels;
    QStringList descriptorLabels;
    descriptorLabels.append("SIFT");
    descriptorLabels.append("SURF");
    descriptorLabels.append("BRIEF");
    descriptorLabels.append("BRISK");
    descriptorLabels.append("FREAK");
    descriptorLabels.append("ORB");
    descriptorLabels.append("AKAZE");
    keyPointLabels.append("SIFT");
    keyPointLabels.append("SURF");
    keyPointLabels.append("FAST");
    keyPointLabels.append("STAR");
    keyPointLabels.append("BRISK");
    keyPointLabels.append("ORB");
    keyPointLabels.append("AKAZE");
    ui.descriptorComboBox->addItems(descriptorLabels);
    ui.keypointComboBox->addItems(keyPointLabels);
    ui.keypointComboBox->setCurrentIndex(1);
    ui.descriptorComboBox->setCurrentIndex(1);
}

void MainWindow::initRobotUI()
{
    QStringList manipulators;
    manipulators.append("KUKA Agilus 1");
    manipulators.append("KUKA Agilus 2");
    ui.robotComboBox->addItems(manipulators);

    homeX = 0.445;
    homeY1 = -0.6025;
    homeY2 = 0.6025;
    homeZ = 1.66;
    homeRoll = 0.0;
    homePitch = 180.0;
    homeYaw = 0.0;
}

void MainWindow::updatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if(runStream){
        viewer1->updatePointCloud(cloud,"sample cloud");
        if(!viewer1->updatePointCloud(cloud,"sample cloud")){
            viewer1->addPointCloud(cloud,"sample cloud");
        }
        w1->update();
        *currentCloud = *cloud;
    }
}

void MainWindow::update2Dimage(QImage image)
{
    saveImage = image;
    ui.label2D->setPixmap(QPixmap::fromImage(image));
    qnode.setImageReading(false);
}

void MainWindow::printToLog(QString text)
{
    QString header = "\n";
    header.append(text);
    ui.logViewer->moveCursor(QTextCursor::End);
    ui.logViewer->insertPlainText(header);
}

void MainWindow::on_saveImageButton_clicked(bool check)
{
    QImage tmpImage = saveImage;
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save Image"), "/home/minions",tr("Image (*.png)"));
    if(fileName.length() != 0) {
        if(!fileName.endsWith(".png",Qt::CaseInsensitive)){
            fileName.append(".png");
        }
        QString tmpstring = "Saving .png to ";
        tmpstring.append(fileName);
        printToLog(tmpstring);
        tmpImage.save(fileName);
    }
}

void MainWindow::on_boxesCheckbox_clicked(bool check){
    drawShapes();
}

void MainWindow::on_framesCheckbox_clicked(bool check){
    drawShapes();
}

void MainWindow::on_runningCheckBox_clicked(bool check)
{
    Q_EMIT setProcessImageRunning(check);
    if(check) {
        printToLog("Image detection running");
    } else {
        printToLog("Image detection not running");
    }
}

void MainWindow::on_colorCheckBox_clicked(bool check)
{
    Q_EMIT setProcessImageColor(check);
    if(check) {
        printToLog("Color image");
    } else {
        printToLog("Grayscale image");
    }
}

void MainWindow::on_undistortCheckBox_clicked(bool check)
{
    Q_EMIT setProcessImageUndistort(check);
    if(check) {
        printToLog("Distorted image");
    } else {
        printToLog("Undistorted image");
    }
}

void MainWindow::on_bruteforceCheckBox_clicked(bool check)
{
    Q_EMIT setProcessImageBruteforce(check);
    if(check) {
        printToLog("Bruteforce matching");
    } else {
        printToLog("FLANN Matching");
    }
}

void MainWindow::on_updateSettingsPushButton_clicked(bool check)
{
    Q_EMIT setProcessImageKeypointDescriptor(ui.keypointComboBox->currentText().toStdString(),
                                             ui.descriptorComboBox->currentText().toStdString());
}

void MainWindow::on_imageToDetectButton_clicked(bool check)
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),"/home/minions",tr("Image (*.png)"));
    if(fileName.length() != 0) {
        QString tmpstring = "Loading matching image from ";
        tmpstring.append(fileName);
        printToLog(tmpstring);
        Q_EMIT setProcessImageMatchingPicture(fileName.toStdString());
    }
}

void MainWindow::on_planTrajPushButton_clicked(bool check)
{
    if(ui.robotComboBox->currentIndex()==0) {
        Q_EMIT plan_ag1(ui.xPosDoubleSpinBox->value()+homeX,
                        ui.yPosDoubleSpinBox->value()+homeY1,
                        ui.zPosDoubleSpinBox->value()+homeZ,
                        ((homeRoll+ui.rollDoubleSpinBox->value())*M_PI)/180.0,
                        ((homePitch+ui.pitchDoubleSpinBox->value())*M_PI)/180.0,
                        ((homeYaw+ui.yawDoubleSpinBox->value())*M_PI)/180.0);

    } else if(ui.robotComboBox->currentIndex()==1) {
        Q_EMIT plan_ag2(ui.xPosDoubleSpinBox->value()+homeX,
                        ui.yPosDoubleSpinBox->value()+homeY2,
                        ui.zPosDoubleSpinBox->value()+homeZ,
                        ((homeRoll+ui.rollDoubleSpinBox->value())*M_PI)/180.0,
                        ((homePitch+ui.pitchDoubleSpinBox->value())*M_PI)/180.0,
                        ((homeYaw+ui.yawDoubleSpinBox->value())*M_PI)/180.0);
    }
}

void MainWindow::on_moveManipPushButton_clicked(bool check)
{
    if(ui.robotComboBox->currentIndex()==0) {
        Q_EMIT move_ag1(ui.xPosDoubleSpinBox->value()+homeX,
                        ui.yPosDoubleSpinBox->value()+homeY1,
                        ui.zPosDoubleSpinBox->value()+homeZ,
                        ((homeRoll+ui.rollDoubleSpinBox->value())*M_PI)/180.0,
                        ((homePitch+ui.pitchDoubleSpinBox->value())*M_PI)/180.0,
                        ((homeYaw+ui.yawDoubleSpinBox->value())*M_PI)/180.0);

    } else if(ui.robotComboBox->currentIndex()==1) {
        Q_EMIT move_ag2(ui.xPosDoubleSpinBox->value()+homeX,
                        ui.yPosDoubleSpinBox->value()+homeY2,
                        ui.zPosDoubleSpinBox->value()+homeZ,
                        ((homeRoll+ui.rollDoubleSpinBox->value())*M_PI)/180.0,
                        ((homePitch+ui.pitchDoubleSpinBox->value())*M_PI)/180.0,
                        ((homeYaw+ui.yawDoubleSpinBox->value())*M_PI)/180.0);
    }
}

void MainWindow::on_homeManipPushButton_clicked(bool check)
{
    ui.xPosDoubleSpinBox->setValue(0.0);
    ui.yPosDoubleSpinBox->setValue(0.0);
    ui.zPosDoubleSpinBox->setValue(0.0);
    ui.rollDoubleSpinBox->setValue(0.0);
    ui.pitchDoubleSpinBox->setValue(0.0);
    ui.yawDoubleSpinBox->setValue(0.0);
}

void MainWindow::on_testPlanButton_clicked(bool check){
    Q_EMIT plan_ag2(partAInTag(0,3),partAInTag(1,3),1.5,0,3.1415,(-23.5)*(M_PI/180.0));
}

void MainWindow::on_testMoveButton_clicked(bool check){
    Q_EMIT move_ag2(partAInTag(0,3),partAInTag(1,3),1.5,0,3.1415,(-23.5)*(M_PI/180.0));
}

void MainWindow::on_plan2DcorrButton_clicked(bool check)
{
    double correctionY = partAInTag(0,3)-qnode.getYoffset();
    double correctionX = partAInTag(1,3)-qnode.getXoffset();
    Q_EMIT plan_ag2(correctionY,correctionX,1.5,0,3.1415,(-23.5)*(M_PI/180.0));
}

void MainWindow::on_move2DcorrButton_clicked(bool check)
{
    double correctionY = partAInTag(0,3)-qnode.getYoffset();
    double correctionX = partAInTag(1,3)-qnode.getXoffset();
    Q_EMIT move_ag2(correctionY,correctionX,1.5,0,3.1415,(-23.5)*(M_PI/180.0));
    partAInTag(0,3) = correctionY;
    partAInTag(1,3) = correctionX;
}

void MainWindow::on_setDepthPushButton_clicked(bool check)
{
    Q_EMIT setProcessImageDepthLambda(ui.lambdaDoubleSpinBox->value());
}

void MainWindow::on_openGripperButton_clicked(bool check)
{
    if(ui.robotComboBox->currentIndex()==0) {
        Q_EMIT openAG1Gripper();
    }
}

void MainWindow::on_closeGripperButton_clicked(bool check)
{
    if(ui.robotComboBox->currentIndex()==0) {
        Q_EMIT closeAG1Gripper();
    }
}

void MainWindow::on_loadPCDButton_clicked(bool check)
{
    runStream = false;
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),"/home/minions",tr("PointCloud (*.pcd)"));
    if(fileName.length() != 0){
        //Non empty string
        QString tmpstring = "Loading .pcd from ";
        tmpstring.append(fileName);
        printToLog(tmpstring);
        if(ui.loadRgbCheckbox->isChecked()){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr displayCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(fileName.toUtf8().constData(), *displayCloud) == -1)
            {
                return;
            }

            displayNewPointCloud(filters->visualize_rgb(displayCloud));
        }
        else{
            pcl::PointCloud<pcl::PointXYZ>::Ptr displayCloud(new pcl::PointCloud<pcl::PointXYZ>);
            if(pcl::io::loadPCDFile<pcl::PointXYZ>(fileName.toUtf8().constData(), *displayCloud) == -1)
            {
                return;
            }
            *currentCloud = *displayCloud;
            displayNewPointCloud(filters->visualize(displayCloud));
        }
    }
}

void MainWindow::on_subscribeToTopicButton_clicked(bool check)
{
    runStream = true;
    Q_EMIT subscribeToPointCloud2(ui.topicComboBox->currentText());
}

void MainWindow::on_subscribeToTopicButton2_clicked(bool check)
{
    Q_EMIT subscribeTo2DobjectDetected(ui.topicComboBox2->currentText());
}

void MainWindow::on_detectObjectsButton_clicked(bool check)
{
    //detect objects here
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr detected(new pcl::PointCloud<pcl::PointXYZRGB>);
    icpResult objectDetectionResult = filters->object_detection(currentCloud,models.at(ui.objectsListA->currentIndex())->getModels(),models.at(ui.objectsListB->currentIndex())->getModels());
    displayNewPointCloud(filters->visualize_rgb(objectDetectionResult.cloud));

    frameA = objectDetectionResult.partAFinal;
    frameB = objectDetectionResult.partBFinal;

    partAInTag = world * worldToTag * tagToCamera * objectDetectionResult.partAFinal;
    partBInTag = world * worldToTag * tagToCamera * objectDetectionResult.partBFinal;

    viewer1->addCoordinateSystem(0.2,frameA);
    viewer1->addCoordinateSystem(0.2,frameB);

    QString partA = "X: ";
    partA.append(QString::number(partAInTag(0,3)));
    partA.append(", Y: ");
    partA.append(QString::number(partAInTag(1,3)));
    partA.append(", Z: ");
    partA.append(QString::number(partAInTag(2,3)));

    QString partB = "X: ";
    partB.append(QString::number(partBInTag(0,3)));
    partB.append(", Y: ");
    partB.append(QString::number(partBInTag(1,3)));
    partB.append(", Z: ");
    partB.append(QString::number(partBInTag(2,3)));

    printToLog("Position of part a in world:");
    printToLog(partA);
    printToLog("Position of part b in world");
    printToLog(partB);

    ui.testPlanButton->setEnabled(true);
    ui.testMoveButton->setEnabled(true);
}

}  // namespace agilus_master_project


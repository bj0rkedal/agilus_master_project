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
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(this,SIGNAL(subscribeToPointCloud2(QString)),&qnode,SLOT(subscribeToPointCloud2(QString)));
    QObject::connect(&qnode,SIGNAL(updatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr)),this,SLOT(updatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr)));
    init();
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
    if(ui.boxesCheckbox->isChecked()){
        viewer1->addCube(-0.510, -0.130, -0.5, 0.3, 0.76, 1.19, 0, 1.0, 0, "partA", 0);
        viewer1->addCube(-0.130, 0.27, -0.5, 0.3, 0.76, 1.19, 1.0, 0, 0, "partB", 0);
    }
    if(ui.framesCheckbox->isChecked()){
        viewer1->addCoordinateSystem(0.2,tag);
        viewer1->addCoordinateSystem(0.2,camera);
        viewer1->addCoordinateSystem(0.5,worldFrame);
    }
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


    QStringList list;
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
    ui.logViewer->insertPlainText("User interface initiated");
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

void MainWindow::printToLog(QString text)
{
    QString header = "\n";
    header.append(text);
    ui.logViewer->moveCursor(QTextCursor::End);
    ui.logViewer->insertPlainText(header);
}

void MainWindow::on_loadPCDButton_clicked(bool check)
{
    runStream = false;
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),"/home/minions",tr("PointCload (*.pcd)"));
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

void MainWindow::on_detectObjectsButton_clicked(bool check)
{
    //detect objects here

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr detected(new pcl::PointCloud<pcl::PointXYZRGB>);
    //detected = filters->object_detection(currentCloud,box->getModels(),box->getModels());
    icpResult objectDetectionResult = filters->object_detection(currentCloud,models.at(ui.objectsListA->currentIndex())->getModels(),models.at(ui.objectsListB->currentIndex())->getModels());
    displayNewPointCloud(filters->visualize_rgb(objectDetectionResult.cloud));


    frameA = objectDetectionResult.partAFinal;
    frameB = objectDetectionResult.partBFinal;



    partAInTag = world * tagToCamera * objectDetectionResult.partAFinal;
    partBInTag = world * tagToCamera * objectDetectionResult.partBFinal;

    viewer1->addCoordinateSystem(0.2,frameA);
    viewer1->addCoordinateSystem(0.2,frameB);


    std::cout << "part a in world: " << std::endl << partAInTag << std::endl;
    std::cout << "part b in world: " << std::endl << partBInTag << std::endl;
}

}  // namespace agilus_master_project


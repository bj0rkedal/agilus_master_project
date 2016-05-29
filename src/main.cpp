//
// Original author Kristoffer Larsen. Latest change date 01.05.2016
// main.cpp instantiataes one instance of the class main_window and launces the application.
//
// Created as part of the software solution for a Master's Thesis in Production Technology at NTNU Trondheim.
//

#include <QtGui>
#include <QApplication>
#include "../include/agilus_master_project/main_window.hpp"

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    agilus_master_project::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();
	return result;
}

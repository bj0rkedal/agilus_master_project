/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/

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

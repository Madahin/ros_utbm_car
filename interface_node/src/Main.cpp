#include <QApplication>
#include <ros/ros.h>
#include "include/Mainwindow.hpp"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MainWindow m(0, argc, argv);

    m.show();

    return a.exec();
}


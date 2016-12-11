#ifndef MAIN_WINDOW
#define MAIN_WINDOW

#include <iostream>

#include <QMainWindow>
#include <QMdiArea>
#include <QString>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <marble/MarbleWidget.h>
#include <marble/MarbleModel.h>
#include <marble/HttpDownloadManager.h>

#include "include/RosThread.hpp"

class MainWindow : public QMainWindow
{
    Q_OBJECT

    private:
        QMdiArea * m_centerArea;
        Marble::MarbleWidget * m_mapView;
        RosThread * m_rosThread;

    public:
        explicit MainWindow(QWidget *parent = 0, int argc=0, char ** argv=nullptr);

    private:
        void InitMarble();
        Q_SLOT void Process_nmea(QString msg);

};

#endif

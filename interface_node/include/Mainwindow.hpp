#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <fstream>
#include <iostream>

#include <QAction>
#include <QtConcurrent/QtConcurrentRun>
#include <QGridLayout>
#include <QLabel>
#include <QMainWindow>
#include <QMdiArea>
#include <QMenu>
#include <QMenuBar>
#include <QPixmap>
#include <QString>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <marble/MarbleWidget.h>
#include <marble/MarbleModel.h>
#include <marble/HttpDownloadManager.h>

#include <boost/filesystem/convenience.hpp>

#include "include/RosThread.hpp"
#include "include/Tools.hpp"

/**
 * @brief The MainWindow class represente our interface
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

    private:
        /////////////////////////////////////////////////
        /// Main
        /////////////////////////////////////////////////
        QMdiArea * m_centerArea;
        RosThread * m_rosThread;
        std::string m_dataDirectory;

        /////////////////////////////////////////////////
        /// GPS
        /////////////////////////////////////////////////
        Marble::MarbleWidget * m_mapView;
        QLabel * m_latError;
        QLabel * m_lonError;
        std::string m_gpsDataFolder;
        std::string m_gpsDataFile;

        /////////////////////////////////////////////////
        /// Camera
        /////////////////////////////////////////////////
        QLabel * m_imageViewer;
        std::string m_cameraDataFolder;
        std::string m_cameraDataFile;
        quint32 m_cameraImageIndex;

    public:
        explicit MainWindow(QWidget *parent = 0, int argc=0, char ** argv=nullptr);

    private:
        /**
         * @brief InitDirectories make sure every directory we need exist
         */
        void InitDirectories();

        /**
         * @brief InitGPS intialise the GPS widget
         */
        void InitGPS();

        /**
         * @brief InitImageViewer initialise the camera widget
         */
        void InitImageViewer();

        /**
         * @brief SaveImage save a picture in a directory
         * @param data the picture we want to save
         * @param filename the path in wich we save te picture
         */
        void SaveImage(const QPixmap& data, const QString& filename) const;

        /**
         * @brief Process_nmea update our map position and save the NMEA trame
         */
        Q_SLOT void Process_nmea(QString msg);

        /**
         * @brief Process_camera update our camera widget and save picture
         */
        Q_SLOT void Process_camera(QPixmap frame);

        /**
         * @brief Close_camera close the image feed
         */
        Q_SLOT void Close_camera();
};

#endif

#include "include/Mainwindow.hpp"

MainWindow::MainWindow(QWidget *parent, int argc, char ** argv) :
    QMainWindow(parent)
{
    m_rosThread = new RosThread(argc, argv);

    if(!m_rosThread->Init())
        close();

    m_centerArea = new QMdiArea;
    setCentralWidget(m_centerArea);

    InitDirectories();
    InitGPS();
    InitImageViewer();
}

void MainWindow::InitDirectories()
{
    m_rosThread->GetNodeHandle().param<std::string>("/ros_utbm_car/data_folder", m_dataDirectory, "/tmp/ros_utbm_car/");
    AppendDirSeparator(m_dataDirectory);

    // GPS
    m_rosThread->GetNodeHandle().param<std::string>("/ros_utbm_car/gps_data_folder", m_gpsDataFolder, "gps/");
    AppendDirSeparator(m_gpsDataFolder);
    m_gpsDataFolder = m_dataDirectory + m_gpsDataFolder;
    boost::filesystem::path gpsDataPath(m_gpsDataFolder);
    boost::filesystem::create_directories(gpsDataPath);
    m_rosThread->GetNodeHandle().param<std::string>("/ros_utbm_car/gps_data_file", m_gpsDataFile, "data.csv");
    m_gpsDataFile = m_gpsDataFolder + m_gpsDataFile;

    // Camera
    m_rosThread->GetNodeHandle().param<std::string>("/ros_utbm_car/gps_data_folder", m_cameraDataFolder, "camera/");
    AppendDirSeparator(m_cameraDataFolder);
    m_cameraDataFolder = m_dataDirectory + m_cameraDataFolder;
    m_rosThread->GetNodeHandle().param<std::string>("/ros_utbm_car/camera_data_file", m_cameraDataFile, "data.csv");
    m_cameraDataFile = m_cameraDataFolder + m_cameraDataFile;
    boost::filesystem::path cameraDataPath(m_cameraDataFolder);
    boost::filesystem::create_directories(cameraDataPath);
    m_cameraImageIndex = -1;
    for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(cameraDataPath), {}))
    {
        bool ok = false;
        quint32 tmp = QString::fromStdString(entry.path().stem().string()).toUInt(&ok);
        if(ok)
        {
            if(tmp > m_cameraImageIndex){
                m_cameraImageIndex = tmp;
            }
        }
    }
    m_cameraImageIndex += 1;
}

void MainWindow::InitGPS()
{
    m_mapView = new Marble::MarbleWidget;
    m_mapView->setWindowTitle("GPS");
    m_mapView->setMinimumSize(640, 480);
    m_mapView->setProjection(Marble::Mercator);
    m_mapView->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
    m_mapView->setShowOverviewMap(false);

    Marble::MarbleModel * model = m_mapView->model();
    model->clearPersistentTileCache();
    std::string dataFilePath;
    m_rosThread->GetNodeHandle().param<std::string>("/ros_utbm_car/osm_data_folder", dataFilePath, "/home/madahin/catkin_ws/src/ros_utbm_car/mapdata/map.osm");
    model->addGeoDataFile(QString::fromStdString(dataFilePath));
    model->downloadManager()->setDownloadEnabled(false);
    model->setHome(6.8666700, 47.6333300, 3250);

    m_mapView->goHome();

    m_centerArea->addSubWindow(m_mapView);

    m_latError = new QLabel("N/A");
    m_lonError = new QLabel("N/A");

    QWidget * dataWidget = new QWidget;
    dataWidget->setWindowTitle("GPS data");

    QGridLayout * dataLayout = new QGridLayout;
    dataLayout->addWidget(new QLabel("Latitude error\t: "), 0, 0);
    dataLayout->addWidget(new QLabel("Longitude error\t: "), 1, 0);
    dataLayout->addWidget(m_latError, 0, 1);
    dataLayout->addWidget(m_lonError, 1, 1);
    dataWidget->setLayout(dataLayout);
    m_centerArea->addSubWindow(dataWidget);

    connect(m_rosThread, SIGNAL(nmeaReceived(QString)), this, SLOT(Process_nmea(QString)));
}

void MainWindow::InitImageViewer()
{
    m_imageViewer = new QLabel;
    m_imageViewer->setAttribute(Qt::WA_DeleteOnClose);
    m_imageViewer->setMinimumSize(640, 480);
    m_imageViewer->setWindowTitle("Camera");
    m_centerArea->addSubWindow(m_imageViewer);

    connect(m_rosThread, SIGNAL(FrameReceived(QPixmap)), this, SLOT(Process_camera(QPixmap)));
    connect(m_imageViewer, SIGNAL(destroyed(QObject*)), this, SLOT(Close_camera()));
}

void MainWindow::Process_nmea(QString msg)
{
    if(m_mapView == nullptr){
        return;
    }

    auto e = GetSplitedEpoch();
#ifndef NDEBUG
    std::cout << e[0] << '\t' << e[1] << '\t' << msg.toStdString() << std::endl;
#endif

    // save message
    std::ofstream file(m_gpsDataFile, std::ios::out | std::ios::app);
    file << e[0] << '\t' << e[1] << '\t' << msg.toStdString();

    // set position
    if(msg.startsWith("$GPGGA"))
    {
        QStringList gpggaParts = msg.split(',');

        QString slatitute = gpggaParts.at(2);
        QString slongitude = gpggaParts.at(4);

        if(!(slatitute.isEmpty() || slongitude.isEmpty())){

            auto coord = ConvertNMEAToDegree(slatitute, gpggaParts.at(3).at(0), slongitude, gpggaParts.at(5).at(0));
            double latitude = coord[0];
            double longitude = coord[1];

            m_mapView->centerOn(longitude, latitude, true);
        }
    }

    if(msg.startsWith("$GPGST"))
    {
        QStringList gpgstParts = msg.split(',');

        QString slatituteError = gpgstParts.at(6);
        QString slongitudeError = gpgstParts.at(7);

        m_latError->setText((slatituteError.isEmpty()) ? "N/A" : QString("%1m").arg(slatituteError.toDouble()));
        m_lonError->setText((slongitudeError.isEmpty()) ? "N/A" : QString("%1m").arg(slongitudeError.toDouble()));
    }
}

void MainWindow::Process_camera(QPixmap frame)
{
    auto e = GetSplitedEpoch();

    // save message
    std::ofstream file(m_cameraDataFile, std::ios::out | std::ios::app);
    file << e[0] << '\t' << e[1] << '\t' << m_cameraImageIndex << std::endl;

    //frame.save(QString("%1%2.png").arg(QString::fromStdString(m_cameraDataFolder)).arg(QString::number(m_cameraImageIndex)));
    QString filename = QString("%1%2.png").arg(QString::fromStdString(m_cameraDataFolder)).arg(QString::number(m_cameraImageIndex));
    QtConcurrent::run(this, &MainWindow::SaveImage, frame, filename);
    m_cameraImageIndex += 1;

    // set position
    m_imageViewer->setPixmap(frame);
}

void MainWindow::SaveImage(const QPixmap& data, const QString& filename) const
{
    data.save(filename);
}

void MainWindow::Close_camera()
{
    disconnect(m_rosThread, SIGNAL(FrameReceived(QPixmap)), this, SLOT(Process_camera(QPixmap)));
}

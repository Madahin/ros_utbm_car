#include "include/Mainwindow.hpp"

MainWindow::MainWindow(QWidget *parent, int argc, char ** argv) :
    QMainWindow(parent)
{
    // Creating our ROS thread
    m_rosThread = new RosThread(argc, argv);

    // Hope it succeded
    if(!m_rosThread->Init())
        close();

    // Initialising our interface
    m_centerArea = new QMdiArea;
    setCentralWidget(m_centerArea);

    // Initialising our compononents
    InitDirectories();
    InitGPS();
    InitImageViewer();
}

void MainWindow::InitDirectories()
{
    // Retreive and sanitize the main folder path
    m_rosThread->GetNodeHandle().param<std::string>("/ros_utbm_car/data_folder", m_dataDirectory, "/tmp/ros_utbm_car/");
    AppendDirSeparator(m_dataDirectory);

    /////////////////
    ///    GPS    ///
    /////////////////

    // Retreive and sanitize the gps folder path
    m_rosThread->GetNodeHandle().param<std::string>("/ros_utbm_car/gps_data_folder", m_gpsDataFolder, "gps/");
    AppendDirSeparator(m_gpsDataFolder);
    m_gpsDataFolder = m_dataDirectory + m_gpsDataFolder;
    // We make sure the directory exist
    boost::filesystem::path gpsDataPath(m_gpsDataFolder);
    boost::filesystem::create_directories(gpsDataPath);
    // We create the path of the file wich will contains our data
    m_rosThread->GetNodeHandle().param<std::string>("/ros_utbm_car/gps_data_file", m_gpsDataFile, "data.csv");
    m_gpsDataFile = m_gpsDataFolder + m_gpsDataFile;

    ////////////////
    ///  Camera  ///
    ////////////////

    // Retreive and sanitize the camera folder path
    m_rosThread->GetNodeHandle().param<std::string>("/ros_utbm_car/camera_data_folder", m_cameraDataFolder, "camera/");
    AppendDirSeparator(m_cameraDataFolder);
    m_cameraDataFolder = m_dataDirectory + m_cameraDataFolder;
    // We make sur the directory exist
    boost::filesystem::path cameraDataPath(m_cameraDataFolder);
    boost::filesystem::create_directories(cameraDataPath);
    // We create the path of the file wich will contains our data
    m_rosThread->GetNodeHandle().param<std::string>("/ros_utbm_car/camera_data_file", m_cameraDataFile, "data.csv");
    m_cameraDataFile = m_cameraDataFolder + m_cameraDataFile;

    // We detect the index of the last image we saved, so we can continue from here.
    m_cameraImageIndex = -1;
    // Iterate over each file in the camera data directory
    for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(cameraDataPath), {}))
    {
        bool ok = false;
        // is the filename without the extension is just a number
        quint32 tmp = QString::fromStdString(entry.path().stem().string()).toUInt(&ok);
        // if yes, we check if it's the largest we saw so far.
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
    // Create a marble widget
    m_mapView = new Marble::MarbleWidget;
    // Set a name
    m_mapView->setWindowTitle("GPS");
    // Set a minimum size, don't do it and you must resize the
    // widget manualy for it to have an usable size (ie. not be ridiculously tiny)
    m_mapView->setMinimumSize(640, 480);
    // Set the prkection type
    m_mapView->setProjection(Marble::Mercator);
    // Set the map theme to open street map
    m_mapView->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
    // deactivate an intusive widget
    m_mapView->setShowOverviewMap(false);

    // the model containing our data
    Marble::MarbleModel * model = m_mapView->model();
    // We don't whant data we didn't downloaded manually
    model->clearPersistentTileCache();
    std::string dataFilePath;
    // Load OSM data
    m_rosThread->GetNodeHandle().param<std::string>("/ros_utbm_car/osm_data_file", dataFilePath, "mapdata/map.osm");
    model->addGeoDataFile(QString::fromStdString(dataFilePath));
    // if we don't have internet connexion, the downloadManager broke down, so we deactivate this functionality
    model->downloadManager()->setDownloadEnabled(false);
    // Center of belfort
    model->setHome(6.8666700, 47.6333300, 3250);

    // set the cursor to our home position
    m_mapView->goHome();

    // Now that the widget is configured, we place it in our main window
    m_centerArea->addSubWindow(m_mapView);

    // We create another widget to hold the error numbers
    QWidget * dataWidget = new QWidget;
    dataWidget->setWindowTitle("GPS data");

    // By default, we don't have data about the error
    m_latError = new QLabel("N/A");
    m_lonError = new QLabel("N/A");

    QGridLayout * dataLayout = new QGridLayout;
    dataLayout->addWidget(new QLabel("Latitude error\t: "), 0, 0);
    dataLayout->addWidget(new QLabel("Longitude error\t: "), 1, 0);
    dataLayout->addWidget(m_latError, 0, 1);
    dataLayout->addWidget(m_lonError, 1, 1);
    dataWidget->setLayout(dataLayout);
    m_centerArea->addSubWindow(dataWidget);

    // Connecting the ros thread to our nmea processor
    connect(m_rosThread, SIGNAL(nmeaReceived(QString)), this, SLOT(Process_nmea(QString)));
}

void MainWindow::InitImageViewer()
{
    // Our image viewer is simply a QLabel
    m_imageViewer = new QLabel;
    m_imageViewer->setAttribute(Qt::WA_DeleteOnClose);
    m_imageViewer->setMinimumSize(640, 480);
    m_imageViewer->setWindowTitle("Camera");
    m_centerArea->addSubWindow(m_imageViewer);

    connect(m_rosThread, SIGNAL(FrameReceived(QPixmap)), this, SLOT(Process_camera(QPixmap)));
    // Don't connect this and the application crash when the camera widget is closed
    connect(m_imageViewer, SIGNAL(destroyed(QObject*)), this, SLOT(Close_camera()));
}

void MainWindow::Process_nmea(QString msg)
{
    auto e = GetSplitedEpoch();
#ifndef NDEBUG
    std::cout << e[0] << '\t' << e[1] << '\t' << msg.toStdString() << std::endl;
#endif

    // save message
    std::ofstream file(m_gpsDataFile, std::ios::out | std::ios::app);
    file << e[0] << '\t' << e[1] << '\t' << msg.toStdString();
    file.close();

    // set map widget position
    if((m_mapView != nullptr) && msg.startsWith("$GPGGA"))
    {
        // an NMEA trame is coma separated
        QStringList gpggaParts = msg.split(',');

        // index 2 and 4 are our latitude and longitude
        QString slatitute = gpggaParts.at(2);
        QString slongitude = gpggaParts.at(4);

        // If the gps don't have a fix, the trame can be empty
        if(!(slatitute.isEmpty() || slongitude.isEmpty())){

            // We need to convert the latitude and longitude to a format marble can understande
            auto coord = ConvertNMEAToDegree(slatitute, gpggaParts.at(3).at(0), slongitude, gpggaParts.at(5).at(0));
            double latitude = coord[0];
            double longitude = coord[1];

            // Center our map to the new position, true mean that we use a transition instead of a teleportation
            m_mapView->centerOn(longitude, latitude, true);
        }
    }

    // GPGST is the NMEA trame that contains error number
    if(msg.startsWith("$GPGST"))
    {
        // Same process as the GPGGA trame
        QStringList gpgstParts = msg.split(',');

        QString slatituteError = gpgstParts.at(6);
        QString slongitudeError = gpgstParts.at(7);

        // If we the trame was empty, we show 'N/A', otherwise, we show the error
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

    // We create our path containing the filename
    QString filename = QString("%1%2.png").arg(QString::fromStdString(m_cameraDataFolder)).arg(QString::number(m_cameraImageIndex));
    // We launch a thread to save the picture
    QtConcurrent::run(this, &MainWindow::SaveImage, frame, filename);
    // next picture index
    m_cameraImageIndex += 1;

    // We set the new image in the widget
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

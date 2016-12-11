#include "include/Mainwindow.hpp"

MainWindow::MainWindow(QWidget *parent, int argc, char ** argv) :
    QMainWindow(parent)
{
    m_centerArea = new QMdiArea;
    setCentralWidget(m_centerArea);

    m_rosThread = new RosThread(argc, argv);

    if(!m_rosThread->Init())
        close();

    connect(m_rosThread, SIGNAL(nmeaReceived(QString)), this, SLOT(Process_nmea(QString)));

    InitMarble();
}

void MainWindow::InitMarble()
{
    m_mapView = new Marble::MarbleWidget;
    m_mapView->setProjection(Marble::Mercator);
    m_mapView->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
    m_mapView->setShowOverviewMap(false);

    Marble::MarbleModel * model = m_mapView->model();
    model->clearPersistentTileCache();
    // TODO: Use ros param
    model->addGeoDataFile("/home/madahin/catkin_ws/src/ros_utbm_car/mapdata/map.osm");
    model->downloadManager()->setDownloadEnabled(false);
    model->setHome(6.8666700f, 47.6333300f, 3250);

    m_mapView->goHome();

    m_centerArea->addSubWindow(m_mapView);
}

void MainWindow::Process_nmea(QString msg)
{
    std::cout << msg.toStdString() << std::endl;
}

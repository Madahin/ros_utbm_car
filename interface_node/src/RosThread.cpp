#include "include/RosThread.hpp"

RosThread::RosThread(int argc, char ** argv) :
    m_argc(argc),
    m_argv(argv)
{
}

RosThread::~RosThread()
{
    // Make sure ROS is destroyed
    if (ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    }

    m_thread->wait();
}

bool RosThread::Init()
{
    // ROS must live in his own thread
    m_thread = new QThread();
    moveToThread(m_thread);

    ros::init(m_argc, m_argv, "ros_utbm_car");

    // Did ROS fail the init ?
    if (!ros::master::check())
        return false;

    // Launch ROS only when the thread is ready
    connect(m_thread, &QThread::started, this, &RosThread::Run);

    // now that ROS is initialised, we can create our node handler
    m_nodeHandler.reset(new ros::NodeHandle);

    ros::start();
    ros::Time::init();

    m_thread->start();
    return true;
}

void RosThread::Run()
{
    // We may need to change these topic in the futur
    m_nmeaListener = m_nodeHandler->subscribe("nmea_chatter", 10, &RosThread::nmeaCallback, this);
    m_cameraListener = m_nodeHandler->subscribe("/usb_cam/image_raw", 10, &RosThread::FrameCallback, this);
    // ROS wait for message
    ros::spin();
}

ros::NodeHandle& RosThread::GetNodeHandle() const
{
    return *m_nodeHandler;
}

void RosThread::nmeaCallback(const std_msgs::String &msg)
{
    boost::mutex::scoped_lock(m_gpsMutex);

    Q_EMIT nmeaReceived(QString::fromStdString(msg.data));
}

void RosThread::FrameCallback(const sensor_msgs::Image &frame)
{
    boost::mutex::scoped_lock(m_cameraMutex);

    cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(frame);

    QPixmap pixmap = QPixmap::fromImage(cvMatToQImage(img.get()->image));

    Q_EMIT FrameReceived(pixmap);
}

QImage  RosThread::cvMatToQImage(const cv::Mat &inMat)
{
    switch ( inMat.type() )
          {
             // 8-bit, 4 channel
             case CV_8UC4:
             {
                QImage image( inMat.data,
                              inMat.cols, inMat.rows,
                              static_cast<int>(inMat.step),
                              QImage::Format_ARGB32 );

                return image;
             }

             // 8-bit, 3 channel
             case CV_8UC3:
             {
                QImage image( inMat.data,
                              inMat.cols, inMat.rows,
                              static_cast<int>(inMat.step),
                              QImage::Format_RGB888 );

                return image.rgbSwapped();
             }

             // 8-bit, 1 channel
             case CV_8UC1:
             {
                static QVector<QRgb>  sColorTable( 256 );

                // only create our color table the first time
                if ( sColorTable.isEmpty() )
                {
                   for ( int i = 0; i < 256; ++i )
                   {
                      sColorTable[i] = qRgb( i, i, i );
                   }
                }

                QImage image( inMat.data,
                              inMat.cols, inMat.rows,
                              static_cast<int>(inMat.step),
                              QImage::Format_Indexed8 );

                image.setColorTable( sColorTable );

                return image;
             }

             default:
                qWarning() << "cvMatToQImage() - cv::Mat image type not handled:" << inMat.type();
                break;
          }

          return QImage();
}

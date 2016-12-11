#include "include/RosThread.hpp"

RosThread::RosThread(int argc, char ** argv) :
    m_argc(argc),
    m_argv(argv)
{
}

RosThread::~RosThread()
{
    if (ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    }

    m_thread->wait();
}

bool RosThread::Init()
{
    m_thread = new QThread();
    moveToThread(m_thread);

    connect(m_thread, &QThread::started, this, &RosThread::Run);

    ros::init(m_argc, m_argv, "ros_utbm_car");

    m_nodeHandler.reset(new ros::NodeHandle);

    if (!ros::master::check())
        return false;

    ros::start();
    ros::Time::init();

    m_thread->start();
    return true;
}

void RosThread::Run()
{
    m_nmeaListener = m_nodeHandler->subscribe("nmea_chatter", 10, &RosThread::nmeaCallback, this);
    ros::spin();
}

void RosThread::nmeaCallback(const std_msgs::String &msg)
{
    Q_EMIT nmeaReceived(QString::fromStdString(msg.data));
}

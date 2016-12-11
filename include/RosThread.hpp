#ifndef ROS_THREAD
#define ROS_THREAD

#include <memory>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <QThread>
#include <QString>

class RosThread : public QObject
{
    Q_OBJECT

    private:
        int m_argc;
        char ** m_argv;

        QThread * m_thread;

        std::unique_ptr<ros::NodeHandle> m_nodeHandler;
        ros::Subscriber m_nmeaListener;
    public:
        RosThread(int argc, char **pArgv);
        ~RosThread();

        bool Init();

        void nmeaCallback(const std_msgs::String &msg);

        Q_SLOT void Run();
        Q_SIGNAL void nmeaReceived(QString msg);

};

#endif

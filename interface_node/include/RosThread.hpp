#ifndef ROS_THREAD_HPP
#define ROS_THREAD_HPP

#include <memory>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>

#include <QDebug>
#include <QThread>
#include <QString>
#include <QPixmap>
#include <QImage>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

#include <boost/thread.hpp>


class RosThread : public QObject
{
    Q_OBJECT

    private:
        int m_argc;
        char ** m_argv;

        QThread * m_thread;

        std::unique_ptr<ros::NodeHandle> m_nodeHandler;
        ros::Subscriber m_nmeaListener;
        ros::Subscriber m_cameraListener;

        boost::mutex::scoped_lock m_cameraMutex, m_gpsMutex;

    public:
        RosThread(int argc, char **pArgv);
        ~RosThread();

        bool Init();

        void nmeaCallback(const std_msgs::String &msg);
        void FrameCallback(const sensor_msgs::Image& frame);

        ros::NodeHandle& GetNodeHandle() const;

        Q_SLOT void Run();

        Q_SIGNAL void nmeaReceived(QString msg);
        Q_SIGNAL void FrameReceived(QPixmap frame);

private:
        QImage  cvMatToQImage(const cv::Mat &inMat);

};

#endif

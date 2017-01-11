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

        // The thread in wich we live on
        QThread * m_thread;

        // Since our node handler must be created after ROS initialisation
        // We put it in a pointer
        std::unique_ptr<ros::NodeHandle> m_nodeHandler;
        ros::Subscriber m_nmeaListener;
        ros::Subscriber m_cameraListener;

        // lock mutex, so we don't have race condition
        boost::mutex::scoped_lock m_cameraMutex, m_gpsMutex;

    public:
        RosThread(int argc, char **pArgv);
        ~RosThread();

        /**
         * @brief Init try to initialise ROS
         * @return true if the initialisation succeded, false otherwise
         */
        bool Init();

        /**
         * @brief nmeaCallback transform an std_msgs::String in a QString to send in a signal
         * @param msg the NMEA trame to be transfered
         */
        void nmeaCallback(const std_msgs::String &msg);

        /**
         * @brief FrameCallback transform an sensor_msgs::Imag in a QPixmap to send in a signal
         * @param frame the picture to be transfered
         */
        void FrameCallback(const sensor_msgs::Image& frame);

        /**
         * @brief GetNodeHandle is used to use ROS param in different class
         * @return a const reference to our node handle
         */
        ros::NodeHandle& GetNodeHandle() const;

        /**
         * @brief Run launch ROS
         */
        Q_SLOT void Run();

        Q_SIGNAL void nmeaReceived(QString msg);
        Q_SIGNAL void FrameReceived(QPixmap frame);

private:
        /**
         * @brief cvMatToQImage convert an opencv material to a QImage
         * @param inMat is our opencv material
         * @return the input picture as a QImage
         */
        QImage  cvMatToQImage(const cv::Mat &inMat);

};

#endif

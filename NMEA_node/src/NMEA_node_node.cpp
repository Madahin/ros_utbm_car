#include <iostream>
#include <array>
#include <cerrno>
#include <cstdlib>
#include <cstdio>
#include <boost/regex.hpp>

#include "unistd.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

const unsigned int BUFFER_SIZE = 4096;
const unsigned int R = 0;
const unsigned int W = 1;

void son_process();
void father_process();

void print_error(const std::string &msg);

int nmea_pipe[2];

static char ** environment = nullptr;

int main(int argc, char ** argv, char ** env)
{
    ros::init(argc, argv, "NMEA_node");

    environment = env;
    pid_t pid;

    if(pipe(nmea_pipe) < 0)
    {
        print_error("Unable to create pipe");
        exit(EXIT_FAILURE);
    }

    pid = fork();

    if(pid < 0)
    {
        print_error("Unable to fork");
        exit(EXIT_FAILURE);
    }

    switch(pid)
    {
        case 0: // Son
            {
                son_process();
                break;
            }
        default: // Father
            {
                father_process();
                break;
            }
    }
    return EXIT_SUCCESS;
}

void son_process()
{
    close(nmea_pipe[R]);
    close(W);
    dup(nmea_pipe[W]);

    char * args[] = {"/usr/bin/gpspipe", "gpspipe", "-r"};

    execve(args[0], &args[1], environment);

    print_error("Execve failed");
    exit(EXIT_FAILURE);
}

void father_process()
{
    close(nmea_pipe[W]);

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("nmea_chatter", 1000);
    ros::Rate loop_rate(10);

    boost::regex expr{"\\$GP.{3}.*"};

    std::array<char, BUFFER_SIZE> buffer;

    while(ros::ok())
    {
        std::size_t n = read(nmea_pipe[R], buffer.data(), BUFFER_SIZE);
        if(n > 0)
        {
            std::string mess(buffer.data());
            //std::cout << mess << std::endl;
            if (boost::regex_match(mess, expr))
            {
                std_msgs::String msg;
                msg.data = mess;
                chatter_pub.publish(msg);
                ROS_INFO("%s", mess);
            }
            buffer.fill(0);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void print_error(const std::string &msg)
{
    std::perror(msg.c_str());
}

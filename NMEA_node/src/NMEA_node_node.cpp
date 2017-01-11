#include <iostream>
#include <array>
#include <cerrno>
#include <cstdlib>
#include <cstdio>
#include <boost/regex.hpp>

#include "unistd.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * @brief BUFFER_SIZE is the size of the buffer wich will contains
 * the data we read from gpspipe
 */
const unsigned int BUFFER_SIZE = 4096;

/**
 * @brief R is an helper var, it's the indice of the output stream of a pipe
 */
const unsigned int R = 0;

/**
 * @brief W is an helper var, it's the indice of the input stream of a pipe
 */
const unsigned int W = 1;

/**
 * @brief son_process is the actual function of the forked process
 */
void son_process();

/**
 * @brief father_process is the actual function of the actuel process
 */
void father_process();

/**
 * @brief print_error is an helper function to print system error
 * @param msg is a custom message we can add to the error.
 */
void print_error(const std::string &msg);

/**
 * @brief nmea_pipe is our pipe wich is used to communicate between our two process
 */
int nmea_pipe[2];

/**
 * @brief environment represent our environment variable
 */
static char ** environment = nullptr;

int main(int argc, char ** argv, char ** env)
{
    ros::init(argc, argv, "NMEA_node");

    // We save our environment in a static variable so we can use it in
    // the forked process
    environment = env;
    pid_t pid;

    // Pipe creation
    if(pipe(nmea_pipe) < 0)
    {
        print_error("Unable to create pipe");
        exit(EXIT_FAILURE);
    }

    // Fork
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
    // We dont need this part of the pipe
    close(nmea_pipe[R]);
    // We redirect our standard output to the pipe
    close(W);
    dup(nmea_pipe[W]);

    // The program we are launching, with is argument
    char * args[] = {"/usr/bin/gpspipe", "gpspipe", "-r"};

    // launch the program
    execve(args[0], &args[1], environment);

    // if execve succeded, we shouldn't be here
    print_error("Execve failed");
    exit(EXIT_FAILURE);
}

void father_process()
{
    // We don't need this part of the pipe
    close(nmea_pipe[W]);

    ros::NodeHandle n;

    // We create a ros topic wich can stock up to 1000 message before overflowing
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("nmea_chatter", 1000);
    // the loop is executed every 10ms
    ros::Rate loop_rate(10);

    // regex used to recognise NMEA trame from gpspipe garbage
    boost::regex expr{"\\$GP.{3}.*"};

    std::array<char, BUFFER_SIZE> buffer;

    while(ros::ok())
    {
        // We read as much data as we can from our pipe
        std::size_t n = read(nmea_pipe[R], buffer.data(), BUFFER_SIZE);
        // if we read something
        if(n > 0)
        {
            // we transform raw data in a string
            std::string mess(buffer.data());
            // check if it's an NMEA trame
            if (boost::regex_match(mess, expr))
            {
                // Send our trame as a ROS message
                std_msgs::String msg;
                msg.data = mess;
                chatter_pub.publish(msg);
                ROS_INFO("%s", mess);
            }
            // clear our buffer
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

#include <iostream>
#include "uvcacquisition.hpp"
#include "uvcvideoproducer.hpp"
#include "ros_thread.hpp"
#include <ros/ros.h>
#include <QCoreApplication>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "flir_boson_driver" );
    //QCoreApplication app(argc,argv);

    std::cout << "hello world." << std::endl;
    UvcAcquisition acq;
    ROSThread ros_thread;
    ros_thread.setUvc( & acq );
    ros_thread.start();
    ros_thread.wait();
    return 0;
}


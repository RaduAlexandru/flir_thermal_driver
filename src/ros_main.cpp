#include <iostream>
#include <ros/ros.h>
#include "boson_node.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "flir_boson_driver" );
    BosonNode boson;
    if ( ! boson.isRunning() )
    {
        ROS_ERROR_STREAM("Thermal data capturing did not start.");
        return -1;
    }
    ros::spin();
    return 0;
}


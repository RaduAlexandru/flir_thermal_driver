#include <iostream>
#include <ros/ros.h>
#include "boson_node.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "flir_thermal_driver_boson" );
    BosonNode boson;
    if ( ! boson.isRunning() )
    {
          ROS_ERROR_STREAM("Thermal data capturing from Boson did not start.");
          return -1;
    }
    ros::spin();
    return 0;
}


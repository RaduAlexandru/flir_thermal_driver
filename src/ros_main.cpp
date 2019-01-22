#include <iostream>
#include <ros/ros.h>
#include "boson_node.hpp"
#include "lepton_node.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "flir_thermal_driver" );
    bool useBoson = false;
    bool useLepton = false;

    ros::NodeHandle p_nh("~");
    p_nh.param<bool>( "useBoson", useBoson, false );
    p_nh.param<bool>( "useLepton", useLepton, true );

    if ( useBoson )
    {
      BosonNode boson;
      if ( ! boson.isRunning() )
      {
          ROS_ERROR_STREAM("Thermal data capturing from Boson did not start.");
          return -1;
      }
    }
    if ( useLepton )
    {
      LeptonNode lepton;
      if ( ! lepton.isRunning() )
      {
          ROS_ERROR_STREAM("Thermal data capturing from Lepton did not start.");
          return -1;
      }
    }
    ros::spin();
    return 0;
}


 
//
// Created by splietke on 13.06.18.
//

#include <ros/ros.h>
#include "ros_thread.hpp"
#include <cv_bridge/cv_bridge.h>
#include <QtGui/QImage>
#include <image_transport/image_transport.h>

ROSThread::ROSThread( )
    : m_uvc(NULL), m_it ( ros::NodeHandle("~" ) )

{
    m_pub = m_it.advertise("camera/image", 1);
}

void ROSThread::setUvc( UvcAcquisition * uvc )
{
    if (m_uvc)
        disconnect( m_uvc, &UvcAcquisition::imageReady, this, &ROSThread::newImage );
    m_uvc = uvc;

    bool success = connect( m_uvc, &UvcAcquisition::imageReady, this, &ROSThread::newImage );
    printf("setting UVC done: ");
    if ( success ) printf("successful.\n");
    else printf("failed.\n");
}

void ROSThread::run()
{
  //ros::Subscriber content_sub = nhp.subscribe("display_content", 5, &ROSThread::DisplayContentCallback, this);
  //ros::Subscriber image_sub = nhp.subscribe("image", 5, &ROSThread::ImageCallback, this);
  while(ros::ok())
  {
      ros::spinOnce();
  //    std::cout << "spinning"<< std::endl;
  }
    //ros::spin();
}

void ROSThread::newImage ( const QImage & img )
{
    std::cout << "got the image, trying to publish." << std::endl;
    cv::Mat cv_img(img.height(), img.width(), CV_8UC3, const_cast<uchar*>(img.bits()), img.bytesPerLine());
    cv_bridge::CvImage msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, cv_img);
    m_pub.publish( msg.toImageMsg() );
}

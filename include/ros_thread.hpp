//
// Created by splietke on 13.06.18.
// modified by quenzel on 26.06.18
//
#ifndef ROS_THREAD_HPP_
#define ROS_THREAD_HPP_

#include <QtCore/QThread>
#include <QImage>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include "uvcacquisition.hpp"
class ROSThread : public QThread
{
    Q_OBJECT

public:
    ROSThread( );
    void init();
    void setUvc( UvcAcquisition * uvc );

public slots:
    void run();
    void newImage( const QImage & );

private:
    image_transport::ImageTransport m_it;
    image_transport::Publisher m_pub;
    UvcAcquisition * m_uvc;
};

#endif //ROS_THREAD_HPP_

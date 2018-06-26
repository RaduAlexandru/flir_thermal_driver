#include "uvcvideoproducer.hpp"
#include "uvcacquisition.hpp"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
UvcVideoProducer::UvcVideoProducer(QObject *parent)
    : QObject(parent)
    //, m_surface(NULL)
    , m_uvc(NULL)
{
}

//void UvcVideoProducer::setVideoSurface(QAbstractVideoSurface *surface)
//{
//    if (m_surface != surface && m_surface && m_surface->isActive()) {
//        m_surface->stop();
//    }
//    m_surface = surface;
//    printf("Surface set. Supported formats:");
//    QList<QVideoFrame::PixelFormat> formats = surface->supportedPixelFormats();
//    for (int i = 0; i < formats.length(); i++)
//        printf(" %d", formats[i]);
//    printf("\n");
//    fflush(stdout);
//    if (m_uvc)
//        m_surface->start(m_uvc->videoFormat());
//    emit surfaceChanged(m_surface);
//}

void UvcVideoProducer::setUvc(UvcAcquisition *uvc)
{
    if (m_uvc)
        disconnect(m_uvc, &UvcAcquisition::frameReady,
                   this, &UvcVideoProducer::onNewVideoContentReceived);

    m_uvc = uvc;
    emit uvcChanged(uvc);

//    if (m_surface)
//    {
//        if (m_surface->isActive()) {
//            m_surface->stop();
//        }
//        QVideoSurfaceFormat new_format = m_surface->nearestFormat(uvc->videoFormat());
//        printf("Surface supports format %d width %d height %d",
//               new_format.pixelFormat(),
//               new_format.frameWidth(), new_format.frameHeight());
//        fflush(stdout);
//        m_surface->start(uvc->videoFormat());
//    }

    bool success = connect(m_uvc, &UvcAcquisition::frameReady,
            this, &UvcVideoProducer::onNewVideoContentReceived);
    printf("setting UVC done: ");
    if ( success ) printf("successful.\n");
    else printf("failed.\n");
}

void UvcVideoProducer::onNewVideoContentReceived(const QVideoFrame &frame)
{
    printf("receiving.\n");

    std::cout << "publishing."<< std::endl;
    QImage img( frame.bits(), frame.width(), frame.height(), frame.bytesPerLine(), QVideoFrame::imageFormatFromPixelFormat( frame.pixelFormat() ) );
    img = img.convertToFormat(QImage::Format_RGB888).rgbSwapped(); // from BGR to RGB

    emit newImage( img ) ;
}

#ifndef BOSON_NODE_HPP_
#define BOSON_NODE_HPP_

#include <libuvc/libuvc.h>
#include <unistd.h>
#include "boson_interface.hpp"
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <vector>

class BosonNode
{
public:
    struct UsbId {
        int vid;
        int pid;
    };

    BosonNode( );
    ~BosonNode( );
    void init();

    void pauseStream();
    void resumeStream();
    bool isRunning() { return m_isRunning; }
    void setVideoFormat(const BosonInterface::VideoFormat &format, const int frameHeight, const int frameWidth );

protected:
    bool m_isRunning = false;
    uvc_context_t *m_ctx;
    uvc_device_t *m_dev;
    uvc_device_handle_t * m_devh;
    uvc_stream_ctrl_t m_ctrl;
    BosonInterface * m_bi;
    std::vector<UsbId> m_ids;
    BosonInterface::VideoFormat m_cur_format;
private:
    image_transport::ImageTransport m_it;
    image_transport::CameraPublisher m_pub;

    static void cb(uvc_frame_t *frame, void *ptr);
};

#endif //BOSON_NODE_HPP_

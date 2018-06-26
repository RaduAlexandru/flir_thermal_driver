#ifndef BOSON_INTERFACE_HPP_
#define BOSON_INTERFACE_HPP_

#include <libuvc/libuvc.h>
extern "C" {
#include "boson_sdk/Client_API.h"
}

#include <functional>
//using namespace std;
//using namespace std::placeholders;

class BosonInterface
{
public:

    enum VideoFormat
    {
        YUV420P,
        RGB24,
        Y16
    };


    BosonInterface( uvc_context_t *ctx,
                    uvc_device_t *dev,
                    uvc_device_handle_t *devh );

    ~BosonInterface();

    const std::string getCameraSerialNumber();

    const std::string getCameraPartNumber();

    const std::string getSensorSerialNumber();

    const std::string getSensorPartNumber();

    const std::string getSoftwareRev();

    float getCameraInternalTempC();

    bool getSupportsHwPseudoColor() const;
    bool getSupportsRadiometry() const;

    void performFfc();

    VideoFormat getDefaultFormat() const { return Y16; } //YUV420P; }
    //VideoFormat getDefaultFormat() const { return YUV420P; }
    int getDefaultHeight() const { return 512; }
    int getDefaultWidth() const { return 640; }

private:

//    template <class T, class W>
//    function<W(void)> bind_get(function<FLR_RESULT(T*)> F)
//    {
//        return bind(&BosonInterface::pget<T, W>, this, F);
//    }

//    template <class T, class W>
//    function<void(W)> bind_set(function<FLR_RESULT(T)> F,
//                               function<void(W)> E)
//    {
//        return bind(&BosonInterface::pset<T, W>, this, F, E, _1);
//    }

//    template <class T, class W>
//    W pget(function<FLR_RESULT(T*)> F)
//    {
//        T var;
//        F(&var);
//        return (W)var;
//    }

//    template <class T, class W>
//    void pset(function<FLR_RESULT(T)> F, function<void(W)> E, W var)
//    {
//        F((T)var);
//        emit E(var);
//    }

    uvc_context_t *m_ctx;
    uvc_device_t *m_dev;
    uvc_device_handle_t *m_devh;
    libusb_device_handle *m_usb_devh;
    uvc_device_descriptor_t *m_desc;
};

#endif // BOSON_INTERFACE_HPP_

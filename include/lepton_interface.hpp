#ifndef LEPTON_INTERFACE_HPP_
#define LEPTON_INTERFACE_HPP_

#include <libuvc/libuvc.h>
#include "lepton_sdk/uvc_sdk.h"
#include "lepton_sdk/LEPTON_Types.h"
#include "lepton_sdk/LEPTON_ErrorCodes.h"
#include "lepton_sdk/LEPTON_SDK.h"
#include "lepton_sdk/LEPTON_OEM.h"
#include "lepton_sdk/LEPTON_RAD.h"
#include "lepton_sdk/LEPTON_SYS.h"
#include "lepton_sdk/LEPTON_VID.h"


#include <functional>
//using namespace std;
//using namespace std::placeholders;

class LeptonInterface
{
public:

    enum VideoFormat
    {
        YUV420P,
        RGB24,
        Y16
    };


    LeptonInterface( uvc_context_t *ctx,
                    uvc_device_t *dev,
                    uvc_device_handle_t *devh );

    ~LeptonInterface();

    const std::string getCameraSerialNumber();

    const std::string getCameraPartNumber();

    const std::string getSensorSerialNumber();

    const std::string getSensorPartNumber();

    const std::string getSoftwareRev() const;

    float getCameraInternalTempC();

    bool getSupportsHwPseudoColor();
    bool getSupportsRadiometry();

    void performFfc();

    VideoFormat getDefaultFormat() const { return Y16; } //YUV420P; }
    //VideoFormat getDefaultFormat() const { return YUV420P; }
    int getDefaultHeight() const { return 120; }
    int getDefaultWidth() const { return 160; }


    LEP_RESULT UVC_GetAttribute(LEP_COMMAND_ID commandID,
                                LEP_ATTRIBUTE_T_PTR attributePtr,
                                LEP_UINT16 attributeWordLength);

    LEP_RESULT UVC_SetAttribute(LEP_COMMAND_ID commandID,
                                LEP_ATTRIBUTE_T_PTR attributePtr,
                                LEP_UINT16 attributeWordLength);

    LEP_RESULT UVC_RunCommand(LEP_COMMAND_ID commandID);

    LEP_CAMERA_PORT_DESC_T_PTR GetPortDescription() { return &m_portDesc; }

    unsigned int getRadSpotmeterObjInKelvinX100();
    void updateSpotmeter();
    bool getSupportsRuntimeAgcChange() const;
    const std::string getPtFirmwareVersion() const;

    const std::string getOemFlirPartNumber();

private:

    int leptonCommandIdToUnitId(LEP_COMMAND_ID commandID);

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

    template <class T, class W>
    W pget(std::function<LEP_RESULT(LEP_CAMERA_PORT_DESC_T_PTR, T*)> F)
    {
        T var;
        F(&m_portDesc, &var);
        return (W)var;
    }

    uvc_context_t *m_ctx;
    uvc_device_t *m_dev;
    uvc_device_handle_t *m_devh;
    //libusb_device_handle *m_usb_devh;
    uvc_device_descriptor_t *m_desc;
    LEP_CAMERA_PORT_DESC_T m_portDesc;
    LEP_RAD_ROI_T m_spotmeterRoi;
    uint64_t serialNumber;
    LEP_OEM_SW_VERSION_T swVers;
    LEP_OEM_PART_NUMBER_T partNumber;
};

#endif // BOSON_INTERFACE_HPP_

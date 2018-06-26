#include "boson_interface.hpp"

extern "C" {
#include "boson_sdk/Client_API.h"
#include "boson_sdk/EnumTypes.h"
#include "boson_sdk/UART_Connector.h"
int boson_example();
}

BosonInterface::BosonInterface(uvc_context_t *ctx,
                               uvc_device_t *dev,
                               uvc_device_handle_t *devh)
  : m_ctx(ctx)
  , m_dev(dev)
  , m_devh(devh)
  , m_usb_devh(uvc_get_libusb_handle(m_devh))
{
    printf("Initializing Boson with UVC backend...\n");

    uvc_get_device_descriptor( m_dev, &m_desc);
    printf("Using %s %s with firmware %s\n", m_desc->manufacturer, m_desc->product, m_desc->serialNumber);

    FLR_RESULT result;

    result = Initialize(m_usb_devh);
    printf("Initialize: 0x%08X\n", result);
    if (result)
    {
        printf("Failed to initialize boson interface\n");
    }
}

BosonInterface::~BosonInterface()
{
    printf("\n\nClosing...\n");
    //Close();

    uvc_free_device_descriptor(m_desc);
}

const std::string BosonInterface::getCameraPartNumber()
{
    FLR_BOSON_PARTNUMBER_T pn;
    bosonGetCameraPN(&pn);
    return std::string((char*)pn.value, sizeof(pn.value));
}

const std::string BosonInterface::getCameraSerialNumber()
{
    uint32_t sn;
    bosonGetCameraSN(&sn);
    return std::to_string(sn);
}

const std::string BosonInterface::getSensorPartNumber()
{

    FLR_BOSON_SENSOR_PARTNUMBER_T pn;
    bosonGetSensorPN(&pn);
    return std::string((char*)pn.value, sizeof(pn.value));
}

const std::string BosonInterface::getSensorSerialNumber()
{
    uint32_t sn;
    bosonGetSensorSN(&sn);
    return std::to_string(sn);
}

const std::string BosonInterface::getSoftwareRev()
{
    uint32_t maj, min, rev;
    bosonGetSoftwareRev(&maj, &min, &rev);
    return std::to_string(maj) + "." + std::to_string(min) + "." + std::to_string(rev);
}


bool BosonInterface::getSupportsHwPseudoColor() const
{
    return true;
}

bool BosonInterface::getSupportsRadiometry() const
{
    return false;
}

float BosonInterface::getCameraInternalTempC()
{
    int16_t temp_c_x10;
    bosonlookupFPATempDegCx10(&temp_c_x10);
    return (float)temp_c_x10 / 10.0f;
}

void BosonInterface::performFfc()
{
    FLR_RESULT result = bosonRunFFC();
    printf("RunFFC:  0x%08X \n", result);
}

#include "lepton_interface.hpp"

//#include "lepton_sdk/uvc_sdk.h"
//#include "lepton_sdk/LEPTON_SDK.h"
//#include "lepton_sdk/LEPTON_OEM.h"
//#include "lepton_sdk/LEPTON_RAD.h"
//#include "lepton_sdk/LEPTON_SYS.h"
//#include "lepton_sdk/LEPTON_VID.h"

#define LEP_CID_AGC_MODULE (0x0100)
#define LEP_CID_OEM_MODULE (0x0800)
#define LEP_CID_RAD_MODULE (0x0E00)
#define LEP_CID_SYS_MODULE (0x0200)
#define LEP_CID_VID_MODULE (0x0300)

typedef enum {
  VC_CONTROL_XU_LEP_AGC_ID = 3,
  VC_CONTROL_XU_LEP_OEM_ID,
  VC_CONTROL_XU_LEP_RAD_ID,
  VC_CONTROL_XU_LEP_SYS_ID,
  VC_CONTROL_XU_LEP_VID_ID,
} VC_TERMINAL_ID;

LeptonInterface::LeptonInterface(uvc_context_t *ctx,
                               uvc_device_t *dev,
                               uvc_device_handle_t *devh)
  : m_ctx(ctx)
  , m_dev(dev)
  , m_devh(devh)
//  , m_usb_devh(uvc_get_libusb_handle(m_devh))
{
    printf("Initializing Lepton SDK with UVC backend...\n");
    uvc_get_device_descriptor(dev, &m_desc);
    printf("Using %s %s with firmware %s\n", m_desc->manufacturer, m_desc->product, m_desc->serialNumber);

    m_portDesc.portID = 0;
    m_portDesc.portType = LEP_CCI_UVC;
    m_portDesc.userPtr = this;
    LEP_OpenPort(m_portDesc.portID,
                 m_portDesc.portType,
                 0,
                 &m_portDesc);
    printf("OK\n");

    const uvc_extension_unit_t *units = uvc_get_extension_units(devh);
    while (units)
    {
        printf("Found extension unit ID %d, controls: %08lx, GUID:", units->bUnitID, units->bmControls);
        for (int i = 0; i < 16; i++)
            printf(" %02x", units->guidExtensionCode[i]);
        printf("\n");
        units = units->next;
    }

    const uvc_format_desc_t *desc = uvc_get_format_descs(devh);
    while (desc != NULL)
    {
        int width, height;
        width = desc->frame_descs[0].wWidth;
        height = desc->frame_descs[0].wHeight;
        printf("width: %i height: %i\n",width,height);
        //m_sensorSize = QSize(width, height);
        //break;
        desc = desc->next;
    }

    //LEP_GetOemSoftwareVersion(&m_portDesc, &swVers);

    LEP_GetRadSpotmeterRoi(&m_portDesc, &m_spotmeterRoi);

    //this->setObjectName("LeptonVariation");

    //m_periodicTimer = new QTimer(this);
    //connect(m_periodicTimer, SIGNAL(timeout()), this, SLOT(updateSpotmeter()));
    //m_periodicTimer->start(1000);
}

LeptonInterface::~LeptonInterface()
{
    printf("\n\nClosing...\n");
    //Close();

    uvc_free_device_descriptor(m_desc);
}

const std::string LeptonInterface::getCameraPartNumber()
{
    LEP_OEM_PART_NUMBER_T pn;
    LEP_GetOemFlirPartNumber(&m_portDesc, &pn);
    return std::string((char*)pn.value, sizeof(pn.value));
}

const std::string LeptonInterface::getCameraSerialNumber()
{
    return std::to_string(pget<uint64_t, uint64_t>(LEP_GetSysFlirSerialNumber));
}

const std::string LeptonInterface::getSensorPartNumber()
{
    LEP_OEM_PART_NUMBER_T pn;
    LEP_GetOemFlirPartNumber(&m_portDesc, &pn);
    //partNumber.value, LEP_SYS_MAX_SERIAL_NUMBER_CHAR_SIZ
    return std::string((char*)pn.value, sizeof(pn.value));
}

const std::string LeptonInterface::getOemFlirPartNumber()
{
    LEP_OEM_PART_NUMBER_T pn;
    LEP_GetOemFlirPartNumber(&m_portDesc, &pn);
    return std::string((char*)pn.value, sizeof(pn.value));
}

const std::string LeptonInterface::getSensorSerialNumber()
{
    return getCameraSerialNumber();
}

const std::string LeptonInterface::getSoftwareRev() const
{
    return std::string(m_desc->serialNumber);
}


const std::string LeptonInterface::getPtFirmwareVersion() const
{
    return std::string(m_desc->serialNumber);
}

bool LeptonInterface::getSupportsHwPseudoColor()
{
    bool containsY16 = getPtFirmwareVersion().find("Y16") != std::string::npos;
    return getSupportsRuntimeAgcChange() || !containsY16;
}

bool LeptonInterface::getSupportsRadiometry()
{
    bool runtimeAgc = getSupportsRuntimeAgcChange();
    bool y16Firmware =  getPtFirmwareVersion().find("Y16") != std::string::npos;
    bool radiometricLepton = (getOemFlirPartNumber().find("500-0763-01") != std::string::npos)
               || (getOemFlirPartNumber().find("500-0771-01") != std::string::npos);
    return (runtimeAgc || y16Firmware) && radiometricLepton;
}

//float LeptonInterface::getCameraInternalTempC()
//{
//    int16_t temp_c_x10;
//    bosonlookupFPATempDegCx10(&temp_c_x10);
//    return (float)temp_c_x10 / 10.0f;
//}

bool LeptonInterface::getSupportsRuntimeAgcChange() const
{
    return !(getPtFirmwareVersion().find("v0") == 0);
}

void LeptonInterface::updateSpotmeter()
{
    //emit radSpotmeterInKelvinX100Changed(); //?
}

unsigned int LeptonInterface::getRadSpotmeterObjInKelvinX100()
{
    LEP_RAD_SPOTMETER_OBJ_KELVIN_T spotmeterObj;
    if (LEP_GetRadSpotmeterObjInKelvinX100(&m_portDesc, &spotmeterObj) == LEP_OK)
        return spotmeterObj.radSpotmeterValue;
    else
        return 0;
}


void LeptonInterface::performFfc()
{
    //LEP_RunOemFFC(&m_portDesc);
    LEP_RunSysFFCNormalization(&m_portDesc);
}

int LeptonInterface::leptonCommandIdToUnitId(LEP_COMMAND_ID commandID)
{
    int unit_id;

    switch (commandID & 0x3f00) // Ignore upper 2 bits including OEM bit
    {
    case LEP_CID_AGC_MODULE:
        unit_id = VC_CONTROL_XU_LEP_AGC_ID;
        break;

    case LEP_CID_OEM_MODULE:
        unit_id = VC_CONTROL_XU_LEP_OEM_ID;
        break;

    case LEP_CID_RAD_MODULE:
        unit_id = VC_CONTROL_XU_LEP_RAD_ID;
        break;

    case LEP_CID_SYS_MODULE:
        unit_id = VC_CONTROL_XU_LEP_SYS_ID;
        break;

    case LEP_CID_VID_MODULE:
        unit_id = VC_CONTROL_XU_LEP_VID_ID;
        break;

    default:
        return LEP_RANGE_ERROR;
    }

    return unit_id;
}

LEP_RESULT LeptonInterface::UVC_GetAttribute(LEP_COMMAND_ID commandID,
                            LEP_ATTRIBUTE_T_PTR attributePtr,
                            LEP_UINT16 attributeWordLength)
{
    int unit_id;
    int control_id;
    int result;

    unit_id = leptonCommandIdToUnitId(commandID);
    if (unit_id < 0)
        return (LEP_RESULT)unit_id;

    control_id = ((commandID & 0x00ff) >> 2) + 1;

    // Size in 16-bit words needs to be in bytes
    attributeWordLength *= 2;

    //QMutexLocker lock(&m_mutex);
    result = uvc_get_ctrl(m_devh, unit_id, control_id, attributePtr, attributeWordLength, UVC_GET_CUR);
    if (result != attributeWordLength)
    {
        printf("UVC_GetAttribute failed: %d", result);
        return LEP_COMM_ERROR_READING_COMM;
    }

    return LEP_OK;
}

LEP_RESULT LeptonInterface::UVC_SetAttribute(LEP_COMMAND_ID commandID,
                            LEP_ATTRIBUTE_T_PTR attributePtr,
                            LEP_UINT16 attributeWordLength)
{
    int unit_id;
    int control_id;
    int result;

    unit_id = leptonCommandIdToUnitId(commandID);
    if (unit_id < 0)
        return (LEP_RESULT)unit_id;

    control_id = ((commandID & 0x00ff) >> 2) + 1;

    // Size in 16-bit words needs to be in bytes
    attributeWordLength *= 2;

    //QMutexLocker lock(&m_mutex);
    result = uvc_set_ctrl(m_devh, unit_id, control_id, attributePtr, attributeWordLength);
    if (result != attributeWordLength)
    {
        printf("UVC_SetAttribute failed: %d", result);
        return LEP_COMM_ERROR_READING_COMM;
    }

    return LEP_OK;
}

LEP_RESULT LeptonInterface::UVC_RunCommand(LEP_COMMAND_ID commandID)
{
    int unit_id;
    int control_id;
    int result;

    unit_id = leptonCommandIdToUnitId(commandID);
    if (unit_id < 0)
        return (LEP_RESULT)unit_id;

    control_id = ((commandID & 0x00ff) >> 2) + 1;

    //QMutexLocker lock(&m_mutex);
    result = uvc_set_ctrl(m_devh, unit_id, control_id, &control_id, 1);
    if (result != 1)
    {
        printf("UVC_RunCommand failed: %d", result);
        return LEP_COMM_ERROR_READING_COMM;
    }

    return LEP_OK;
}

/* --------------------------------------------------------------------- */
/* -------- static wrapper functions for use by Lepton SDK only -------- */
/* --------------------------------------------------------------------- */

LEP_RESULT UVC_GetAttribute(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                            LEP_COMMAND_ID commandID,
                            LEP_ATTRIBUTE_T_PTR attributePtr,
                            LEP_UINT16 attributeWordLength)
{
    return static_cast<LeptonInterface*>(portDescPtr->userPtr) ->
            UVC_GetAttribute(commandID, attributePtr, attributeWordLength);
}

LEP_RESULT UVC_SetAttribute(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                            LEP_COMMAND_ID commandID,
                            LEP_ATTRIBUTE_T_PTR attributePtr,
                            LEP_UINT16 attributeWordLength)
{
    return static_cast<LeptonInterface*>(portDescPtr->userPtr) ->
            UVC_SetAttribute(commandID, attributePtr, attributeWordLength);
}

LEP_RESULT UVC_RunCommand(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                          LEP_COMMAND_ID commandID)
{
    return static_cast<LeptonInterface*>(portDescPtr->userPtr) ->
            UVC_RunCommand(commandID);
}

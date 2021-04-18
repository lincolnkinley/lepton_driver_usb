
#include "LEPTON_SDK.h"
#include "uvc_sdk.h"

#define LEP_CID_AGC_MODULE (0x0100)
#define LEP_CID_OEM_MODULE (0x0800)
#define LEP_CID_RAD_MODULE (0x0E00)
#define LEP_CID_SYS_MODULE (0x0200)
#define LEP_CID_VID_MODULE (0x0300)

typedef enum
{
    VC_CONTROL_XU_LEP_AGC_ID = 3,
    VC_CONTROL_XU_LEP_OEM_ID,
    VC_CONTROL_XU_LEP_RAD_ID,
    VC_CONTROL_XU_LEP_SYS_ID,
    VC_CONTROL_XU_LEP_VID_ID,
} VC_TERMINAL_ID;


int leptonCommandIdToUnitId(LEP_COMMAND_ID commandID)
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
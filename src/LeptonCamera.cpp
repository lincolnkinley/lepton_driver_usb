
#include <functional>
#include <iostream>

#include "opencv2/core.hpp"

#include <lepton_sdk/LEPTON_AGC.h>
#include "lepton_sdk/LEPTON_OEM.h"
#include "lepton_sdk/LEPTON_RAD.h"
#include "lepton_sdk/LEPTON_SDK.h"
#include "lepton_sdk/LEPTON_SYS.h"
#include "lepton_sdk/LEPTON_VID.h"
#include "lepton_sdk/LEPTON_Types.h"
#include "lepton_driver/LeptonCamera.hpp"

LeptonCamera::LeptonCamera(uint16_t port_id, LeptonCaptureBits capture_bits,
                           std::function<void(uvc_frame_t *)> callback, bool verbose) : _uvc_manager(
        LEPTON_VID, LEPTON_PID, LEPTON_RESOLUTION_WIDTH, LEPTON_RESOLUTION_HEIGHT,
        static_cast<uvc_frame_format>(capture_bits), callback,
        verbose)
{
    _camera_port_descriptor.portID = port_id;
    _camera_port_descriptor.portType = LEP_CCI_UVC;
    _camera_port_descriptor.userPtr = &_uvc_manager;
    LEP_RESULT result = LEP_OpenPort(_camera_port_descriptor.portID, _camera_port_descriptor.portType, 0,
                                     &_camera_port_descriptor);
    if (result == 0)
    {
        throw;
    }
}

LeptonCamera::~LeptonCamera() noexcept
{
    LEP_RESULT result = LEP_ClosePort(&_camera_port_descriptor);
}

cv::Mat LeptonCamera::GetLastFrame()
{
    return _uvc_manager.GetLastFrame();
}

bool LeptonCamera::SetCaptureBits(LeptonCaptureBits input)
{
    return _uvc_manager.SetVideoFormat(LEPTON_RESOLUTION_WIDTH, LEPTON_RESOLUTION_HEIGHT,
                                       static_cast<uvc_frame_format>(input));
}

/***************************************************************************************************
 *
 *      Lepton SKD AGC Wrapper Functions
 *
 **************************************************************************************************/

LEP_AGC_ENABLE_E LeptonCamera::GetAgcEnableState()
{
    LEP_AGC_ENABLE_E data;
    LEP_GetAgcEnableState(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetAgcEnableState(LEP_AGC_ENABLE_E input)
{
    return LEP_SetAgcEnableState(&_camera_port_descriptor, input);
}

LEP_AGC_POLICY_E LeptonCamera::GetAgcPolicy()
{
    LEP_AGC_POLICY_E data;
    LEP_GetAgcPolicy(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetAgcPolicy(LEP_AGC_POLICY_E agcPolicy)
{
    return LEP_SetAgcPolicy(&_camera_port_descriptor, agcPolicy);
}

LEP_AGC_ROI_T LeptonCamera::GetAgcROI()
{
    LEP_AGC_ROI_T data;
    LEP_GetAgcROI(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetAgcROI(LEP_AGC_ROI_T agcROI)
{
    return LEP_SetAgcROI(&_camera_port_descriptor, agcROI);
}

LEP_AGC_HISTOGRAM_STATISTICS_T LeptonCamera::GetAgcHistogramStatistics()
{
    LEP_AGC_HISTOGRAM_STATISTICS_T data;
    LEP_AGC_HISTOGRAM_STATISTICS_T_PTR data_ptr = &data;
    LEP_GetAgcHistogramStatistics(&_camera_port_descriptor, &data_ptr);
    return data;
}

LEP_UINT16 LeptonCamera::GetAgcLinearHistogramTailSize()
{
    LEP_UINT16 data;
    LEP_GetAgcLinearHistogramTailSize(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetAgcLinearHistogramTailSize(LEP_UINT16 agcLinearHistogramTailSize)
{
    return LEP_SetAgcLinearHistogramTailSize(&_camera_port_descriptor, agcLinearHistogramTailSize);
}

LEP_UINT16 LeptonCamera::GetAgcLinearHistogramClipPercent()
{
    LEP_UINT16 data;
    LEP_GetAgcLinearHistogramClipPercent(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetAgcLinearHistogramClipPercent(LEP_UINT16 agcLinearClipPercent)
{
    return LEP_SetAgcLinearHistogramClipPercent(&_camera_port_descriptor, agcLinearClipPercent);
}

LEP_UINT16 LeptonCamera::GetAgcLinearMaxGain()
{
    LEP_UINT16 data;
    LEP_GetAgcLinearMaxGain(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetAgcLinearMaxGain(LEP_UINT16 agcLinearMaxGain)
{
    return LEP_SetAgcLinearMaxGain(&_camera_port_descriptor, agcLinearMaxGain);
}

LEP_UINT16 LeptonCamera::GetAgcLinearMidPoint()
{
    LEP_UINT16 data;
    LEP_GetAgcLinearMidPoint(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetAgcLinearMidPoint(LEP_UINT16 agcLinearMidPoint)
{
    return LEP_SetAgcLinearMidPoint(&_camera_port_descriptor, agcLinearMidPoint);
}

LEP_UINT16 LeptonCamera::GetAgcLinearDampeningFactor()
{
    LEP_UINT16 data;
    LEP_GetAgcLinearDampeningFactor(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetAgcLinearDampeningFactor(LEP_UINT16 agcLinearDampeningFactor)
{
    return LEP_SetAgcLinearDampeningFactor(&_camera_port_descriptor, agcLinearDampeningFactor);
}

LEP_UINT16 LeptonCamera::GetAgcHeqDampingFactor()
{
    LEP_UINT16 data;
    LEP_GetAgcHeqDampingFactor(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetAgcHeqDampingFactor(LEP_UINT16 agcHeqDampingFactor)
{
    return LEP_SetAgcHeqDampingFactor(&_camera_port_descriptor, agcHeqDampingFactor);
}

LEP_UINT16 LeptonCamera::GetAgcHeqMaxGain()
{
    LEP_UINT16 data;
    LEP_GetAgcHeqMaxGain(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetAgcHeqMaxGain(LEP_UINT16 agcHeqMaxGain)
{
    return LEP_SetAgcHeqMaxGain(&_camera_port_descriptor, agcHeqMaxGain);
}

LEP_UINT16 LeptonCamera::GetAgcHeqClipLimitHigh()
{
    LEP_UINT16 data;
    LEP_GetAgcHeqClipLimitHigh(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetAgcHeqClipLimitHigh(LEP_UINT16 agcHeqClipLimitHigh)
{
    return LEP_SetAgcHeqClipLimitHigh(&_camera_port_descriptor, agcHeqClipLimitHigh);
}

LEP_UINT16 LeptonCamera::GetAgcHeqClipLimitLow()
{
    LEP_UINT16 data;
    LEP_GetAgcHeqClipLimitLow(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetAgcHeqClipLimitLow(LEP_UINT16 agcHeqClipLimitLow)
{
    return LEP_SetAgcHeqClipLimitLow(&_camera_port_descriptor, agcHeqClipLimitLow);
}

LEP_UINT16 LeptonCamera::GetAgcHeqBinExtension()
{
    LEP_UINT16 data;
    LEP_GetAgcHeqBinExtension(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetAgcHeqBinExtension(LEP_UINT16 agcHeqBinExtension)
{
    return LEP_SetAgcHeqBinExtension(&_camera_port_descriptor, agcHeqBinExtension);
}

LEP_UINT16 LeptonCamera::GetAgcHeqMidPoint()
{
    LEP_UINT16 data;
    LEP_GetAgcHeqMidPoint(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetAgcHeqMidPoint(LEP_UINT16 agcHeqMidPoint)
{
    return LEP_SetAgcHeqMidPoint(&_camera_port_descriptor, agcHeqMidPoint);
}

LEP_AGC_HEQ_EMPTY_COUNT_T LeptonCamera::GetAgcHeqEmptyCount()
{
    LEP_AGC_HEQ_EMPTY_COUNT_T data;
    LEP_GetAgcHeqEmptyCount(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetAgcHeqEmptyCount(LEP_AGC_HEQ_EMPTY_COUNT_T emptyCount)
{
    return LEP_SetAgcHeqEmptyCount(&_camera_port_descriptor, emptyCount);
}

LEP_AGC_HEQ_NORMALIZATION_FACTOR_T LeptonCamera::GetAgcHeqNormalizationFactor()
{
    LEP_AGC_HEQ_NORMALIZATION_FACTOR_T data;
    LEP_GetAgcHeqNormalizationFactor(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetAgcHeqNormalizationFactor(LEP_AGC_HEQ_NORMALIZATION_FACTOR_T normalizationFactor)
{
    return LEP_SetAgcHeqNormalizationFactor(&_camera_port_descriptor, normalizationFactor);
}

LEP_AGC_HEQ_SCALE_FACTOR_E LeptonCamera::GetAgcHeqScaleFactor()
{
    LEP_AGC_HEQ_SCALE_FACTOR_E data;
    LEP_GetAgcHeqScaleFactor(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetAgcHeqScaleFactor(LEP_AGC_HEQ_SCALE_FACTOR_E scaleFactor)
{
    return LEP_SetAgcHeqScaleFactor(&_camera_port_descriptor, scaleFactor);
}

LEP_AGC_ENABLE_E LeptonCamera::GetAgcCalcEnableState()
{
    LEP_AGC_ENABLE_E data;
    LEP_GetAgcCalcEnableState(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetAgcCalcEnableState(LEP_AGC_ENABLE_E agcCalculationEnableState)
{
    return LEP_SetAgcCalcEnableState(&_camera_port_descriptor, agcCalculationEnableState);
}

LEP_UINT16 LeptonCamera::GetAgcHeqLinearPercent()
{
    LEP_UINT16 data;
    LEP_GetAgcHeqLinearPercent(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetAgcHeqLinearPercent(LEP_UINT16 agcHeqLinearPercent)
{
    return LEP_SetAgcHeqLinearPercent(&_camera_port_descriptor, agcHeqLinearPercent);
}

/***************************************************************************************************
 *
 *      Lepton SKD OEM Wrapper Functions
 *
 **************************************************************************************************/

LEP_RESULT LeptonCamera::RunOemPowerDown()
{
    return LEP_RunOemPowerDown(&_camera_port_descriptor);
}

LEP_RESULT LeptonCamera::RunOemPowerOn()
{
    // LEP_RunOemPowerOn attempts to access a register which is not possible with a UVC connection, perhaps modifying
    // the SDK would allow this, but for the time being this function is not supported.
    std::cout << "LeptonCamera::RunOemPowerOn() was called, but the function is not supported over UVC.\n";
    return LEP_FUNCTION_NOT_SUPPORTED;
    return LEP_RunOemPowerOn(&_camera_port_descriptor);
}

LEP_RESULT LeptonCamera::RunOemStandby()
{
    return LEP_RunOemStandby(&_camera_port_descriptor);
}

LEP_RESULT LeptonCamera::RunOemReboot()
{
    return LEP_RunOemReboot(&_camera_port_descriptor);
}

LEP_RESULT LeptonCamera::RunOemLowPowerMode1()
{
    return LEP_RunOemLowPowerMode1(&_camera_port_descriptor);
}

LEP_RESULT LeptonCamera::RunOemLowPowerMode2()
{
    return LEP_RunOemLowPowerMode2(&_camera_port_descriptor);
}

LEP_RESULT LeptonCamera::RunOemBit()
{
    return LEP_RunOemBit(&_camera_port_descriptor);
}

LEP_OEM_MASK_REVISION_T LeptonCamera::GetOemMaskRevision()
{
    LEP_OEM_MASK_REVISION_T data;
    LEP_GetOemMaskRevision(&_camera_port_descriptor, &data);
    return data;
}

LEP_OEM_PART_NUMBER_T LeptonCamera::GetOemFlirPartNumber()
{
    LEP_OEM_PART_NUMBER_T data;
    LEP_GetOemFlirPartNumber(&_camera_port_descriptor, &data);
    return data;
}

LEP_OEM_PART_NUMBER_T LeptonCamera::GetOemCustPartNumber()
{
    LEP_OEM_PART_NUMBER_T data;
    LEP_GetOemCustPartNumber(&_camera_port_descriptor, &data);
    return data;
}

LEP_OEM_SW_VERSION_T LeptonCamera::GetOemSoftwareVersion()
{
    LEP_OEM_SW_VERSION_T data;
    LEP_GetOemSoftwareVersion(&_camera_port_descriptor, &data);
    return data;
}

LEP_OEM_VIDEO_OUTPUT_ENABLE_E LeptonCamera::GetOemVideoOutputEnable()
{
    LEP_OEM_VIDEO_OUTPUT_ENABLE_E data;
    LEP_GetOemVideoOutputEnable(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetOemVideoOutputEnable(LEP_OEM_VIDEO_OUTPUT_ENABLE_E oemVideoOutputEnable)
{
    return LEP_SetOemVideoOutputEnable(&_camera_port_descriptor, oemVideoOutputEnable);
}

LEP_OEM_VIDEO_OUTPUT_FORMAT_E LeptonCamera::GetOemVideoOutputFormat()
{
    LEP_OEM_VIDEO_OUTPUT_FORMAT_E data;
    LEP_GetOemVideoOutputFormat(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetOemVideoOutputFormat(LEP_OEM_VIDEO_OUTPUT_FORMAT_E oemVideoOutputFormat)
{
    return LEP_SetOemVideoOutputFormat(&_camera_port_descriptor, oemVideoOutputFormat);
}

LEP_OEM_VIDEO_OUTPUT_SOURCE_E LeptonCamera::GetOemVideoOutputSource()
{
    LEP_OEM_VIDEO_OUTPUT_SOURCE_E data;
    LEP_GetOemVideoOutputSource(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetOemVideoOutputSource(LEP_OEM_VIDEO_OUTPUT_SOURCE_E oemVideoOutputSource)
{
    return LEP_SetOemVideoOutputSource(&_camera_port_descriptor, oemVideoOutputSource);
}

LEP_UINT16 LeptonCamera::GetOemVideoOutputSourceConstant()
{
    LEP_UINT16 data;
    LEP_GetOemVideoOutputSourceConstant(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetOemVideoOutputSourceConstant(LEP_UINT16 oemVideoOutputSourceConstant)
{
    return LEP_SetOemVideoOutputSourceConstant(&_camera_port_descriptor, oemVideoOutputSourceConstant);
}

LEP_OEM_VIDEO_OUTPUT_CHANNEL_E LeptonCamera::GetOemVideoOutputChannel()
{
    LEP_OEM_VIDEO_OUTPUT_CHANNEL_E data;
    LEP_GetOemVideoOutputChannel(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetOemVideoOutputChannel(LEP_OEM_VIDEO_OUTPUT_CHANNEL_E oemVideoOutputChannel)
{
    return LEP_SetOemVideoOutputChannel(&_camera_port_descriptor, oemVideoOutputChannel);
}

LEP_OEM_VIDEO_GAMMA_ENABLE_E LeptonCamera::GetOemVideoGammaEnable()
{
    LEP_OEM_VIDEO_GAMMA_ENABLE_E data;
    LEP_GetOemVideoGammaEnable(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetOemVideoGammaEnable(LEP_OEM_VIDEO_GAMMA_ENABLE_E oemVideoGammaEnable)
{
    return LEP_SetOemVideoGammaEnable(&_camera_port_descriptor, oemVideoGammaEnable);
}

LEP_OEM_STATUS_E LeptonCamera::GetOemCalStatus()
{
    LEP_OEM_STATUS_E data;
    LEP_GetOemCalStatus(&_camera_port_descriptor, &data);
    return data;
}

LEP_OEM_FFC_NORMALIZATION_TARGET_T LeptonCamera::GetOemFFCNormalizationTarget()
{
    LEP_OEM_FFC_NORMALIZATION_TARGET_T data;
    LEP_GetOemFFCNormalizationTarget(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetOemFFCNormalizationTarget(LEP_OEM_FFC_NORMALIZATION_TARGET_T ffcTarget)
{
    return LEP_SetOemFFCNormalizationTarget(&_camera_port_descriptor, ffcTarget);
}

LEP_RESULT LeptonCamera::RunOemFFCNormalization(LEP_OEM_FFC_NORMALIZATION_TARGET_T ffcTarget)
{
    return LEP_RunOemFFCNormalization(&_camera_port_descriptor, ffcTarget);
}

LEP_OEM_FRAME_AVERAGE_T LeptonCamera::GetOemFrameMean()
{
    LEP_OEM_FRAME_AVERAGE_T data;
    LEP_GetOemFrameMean(&_camera_port_descriptor, &data);
    return data;
}

LEP_OEM_POWER_STATE_E LeptonCamera::GetOemPowerMode()
{
    LEP_OEM_POWER_STATE_E data;
    LEP_GetOemPowerMode(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetOemPowerMode(LEP_OEM_POWER_STATE_E powerMode)
{
    return LEP_SetOemPowerMode(&_camera_port_descriptor, powerMode);
}

LEP_RESULT LeptonCamera::RunOemFFC()
{
    return LEP_RunOemFFC(&_camera_port_descriptor);
}

LEP_OEM_GPIO_MODE_E LeptonCamera::GetOemGpioMode()
{
    LEP_OEM_GPIO_MODE_E data;
    LEP_GetOemGpioMode(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetOemGpioMode(LEP_OEM_GPIO_MODE_E gpioMode)
{
    return LEP_SetOemGpioMode(&_camera_port_descriptor, gpioMode);
}

LEP_OEM_VSYNC_DELAY_E LeptonCamera::GetOemGpioVsyncPhaseDelay()
{
    LEP_OEM_VSYNC_DELAY_E data;
    LEP_GetOemGpioVsyncPhaseDelay(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetOemGpioVsyncPhaseDelay(LEP_OEM_VSYNC_DELAY_E numHsyncLines)
{
    return LEP_SetOemGpioVsyncPhaseDelay(&_camera_port_descriptor, numHsyncLines);
}

LEP_OEM_USER_PARAMS_STATE_E LeptonCamera::GetOemUserDefaultsState()
{
    LEP_OEM_USER_PARAMS_STATE_E data;
    LEP_GetOemUserDefaultsState(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::RunOemUserDefaultsCopyToOtp()
{
    return LEP_RunOemUserDefaultsCopyToOtp(&_camera_port_descriptor);
}

LEP_RESULT LeptonCamera::RunOemUserDefaultsRestore()
{
    return LEP_RunOemUserDefaultsRestore(&_camera_port_descriptor);
}

LEP_OEM_THERMAL_SHUTDOWN_ENABLE_T LeptonCamera::GetOemThermalShutdownEnable()
{
    LEP_OEM_THERMAL_SHUTDOWN_ENABLE_T data;
    LEP_GetOemThermalShutdownEnable(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetOemThermalShutdownEnable(LEP_OEM_THERMAL_SHUTDOWN_ENABLE_T ThermalShutdownEnableState)
{
    return LEP_SetOemThermalShutdownEnable(&_camera_port_descriptor, ThermalShutdownEnableState);
}

LEP_OEM_SHUTTER_PROFILE_OBJ_T LeptonCamera::GetOemShutterProfileObj()
{
    LEP_OEM_SHUTTER_PROFILE_OBJ_T data;
    LEP_GetOemShutterProfileObj(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetOemShutterProfileObj(LEP_OEM_SHUTTER_PROFILE_OBJ_T ShutterProfileObj)
{
    return LEP_SetOemShutterProfileObj(&_camera_port_descriptor, ShutterProfileObj);
}

LEP_OEM_BAD_PIXEL_REPLACE_CONTROL_T LeptonCamera::GetOemBadPixelReplaceControl()
{
    LEP_OEM_BAD_PIXEL_REPLACE_CONTROL_T data;
    LEP_GetOemBadPixelReplaceControl(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetOemBadPixelReplaceControl(LEP_OEM_BAD_PIXEL_REPLACE_CONTROL_T BadPixelReplaceControl)
{
    return LEP_SetOemBadPixelReplaceControl(&_camera_port_descriptor, BadPixelReplaceControl);
}

LEP_OEM_TEMPORAL_FILTER_CONTROL_T LeptonCamera::GetOemTemporalFilterControl()
{
    LEP_OEM_TEMPORAL_FILTER_CONTROL_T data;
    LEP_GetOemTemporalFilterControl(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetOemTemporalFilterControl(LEP_OEM_TEMPORAL_FILTER_CONTROL_T TemporalFilterControl)
{
    return LEP_SetOemTemporalFilterControl(&_camera_port_descriptor, TemporalFilterControl);
}


LEP_OEM_COLUMN_NOISE_ESTIMATE_CONTROL_T LeptonCamera::GetOemColumnNoiseEstimateControl()
{
    LEP_OEM_COLUMN_NOISE_ESTIMATE_CONTROL_T data;
    LEP_GetOemColumnNoiseEstimateControl(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT
LeptonCamera::SetOemColumnNoiseEstimateControl(LEP_OEM_COLUMN_NOISE_ESTIMATE_CONTROL_T ColumnNoiseEstimateControl)
{
    return LEP_SetOemColumnNoiseEstimateControl(&_camera_port_descriptor, ColumnNoiseEstimateControl);
}


LEP_OEM_PIXEL_NOISE_SETTINGS_T LeptonCamera::GetOemPixelNoiseSettings()
{
    LEP_OEM_PIXEL_NOISE_SETTINGS_T data;
    LEP_GetOemPixelNoiseSettings(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetOemPixelNoiseSettings(LEP_OEM_PIXEL_NOISE_SETTINGS_T PixelNoiseEstimateControl)
{
    return LEP_SetOemPixelNoiseSettings(&_camera_port_descriptor, PixelNoiseEstimateControl);
}

/***************************************************************************************************
 *
 *      Lepton SKD RAD Wrapper Functions
 *
 **************************************************************************************************/

LEP_RAD_TS_MODE_E LeptonCamera::GetRadTShutterMode()
{
    LEP_RAD_TS_MODE_E data;
    LEP_GetRadTShutterMode(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadTShutterMode(LEP_RAD_TS_MODE_E radTShutterMode)
{
    return LEP_SetRadTShutterMode(&_camera_port_descriptor, radTShutterMode);
}

LEP_RAD_KELVIN_T LeptonCamera::GetRadTShutter()
{
    LEP_RAD_KELVIN_T data;
    LEP_GetRadTShutter(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadTShutter(LEP_RAD_KELVIN_T radTShutter)
{
    return LEP_SetRadTShutter(&_camera_port_descriptor, radTShutter);
}

LEP_RESULT LeptonCamera::RunRadFFC()
{
    return LEP_RunRadFFC(&_camera_port_descriptor);
}

LEP_RBFO_T LeptonCamera::GetRadRBFOInternal0()
{
    LEP_RBFO_T data;
    LEP_GetRadRBFOInternal0(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadRBFOInternal0(LEP_RBFO_T radRBFO)
{
    return LEP_SetRadRBFOInternal0(&_camera_port_descriptor, &radRBFO);
}

LEP_RBFO_T LeptonCamera::GetRadRBFOExternal0()
{
    LEP_RBFO_T data;
    LEP_GetRadRBFOExternal0(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadRBFOExternal0(LEP_RBFO_T radRBFO)
{
    return LEP_SetRadRBFOExternal0(&_camera_port_descriptor, &radRBFO);
}

LEP_RAD_RS_T LeptonCamera::GetRadResponsivityShift()
{
    LEP_RAD_RS_T data;
    LEP_GetRadResponsivityShift(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadResponsivityShift(LEP_RAD_RS_T radResponsivityShift)
{
    return LEP_SetRadResponsivityShift(&_camera_port_descriptor, radResponsivityShift);
}

LEP_RAD_FNUMBER_T LeptonCamera::GetRadFNumber()
{
    LEP_RAD_FNUMBER_T data;
    LEP_GetRadFNumber(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadFNumber(LEP_RAD_FNUMBER_T radFNumber)
{
    return LEP_SetRadFNumber(&_camera_port_descriptor, radFNumber);
}

LEP_RAD_TAULENS_T LeptonCamera::GetRadTauLens()
{
    LEP_RAD_TAULENS_T data;
    LEP_GetRadTauLens(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadTauLens(LEP_RAD_TAULENS_T radTauLens)
{
    return LEP_SetRadTauLens(&_camera_port_descriptor, radTauLens);
}

LEP_RAD_RADIOMETRY_FILTER_T LeptonCamera::GetRadRadometryFilter()
{
    LEP_RAD_RADIOMETRY_FILTER_T data;
    LEP_GetRadRadometryFilter(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadRadometryFilter(LEP_RAD_RADIOMETRY_FILTER_T radRadiometryFilter)
{
    return LEP_SetRadRadometryFilter(&_camera_port_descriptor, radRadiometryFilter);
}

LEP_RAD_LUT256_T LeptonCamera::GetRadTFpaCLut()
{
    LEP_RAD_LUT256_T data;
    LEP_GetRadTFpaCLut(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadTFpaCLut(LEP_RAD_LUT256_T radTFpaCLut)
{
    return LEP_SetRadTFpaCLut(&_camera_port_descriptor, &radTFpaCLut);
}

LEP_RAD_LUT256_T LeptonCamera::GetRadTAuxCLut()
{
    LEP_RAD_LUT256_T data;
    LEP_GetRadTAuxCLut(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadTAuxCLut(LEP_RAD_LUT256_T radTAuxCLut)
{
    return LEP_SetRadTAuxCLut(&_camera_port_descriptor, &radTAuxCLut);
}

LEP_RAD_LUT256_T LeptonCamera::GetRadTFpaLut()
{
    LEP_RAD_LUT256_T data;
    LEP_GetRadTFpaLut(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadTFpaLut(LEP_RAD_LUT256_T radTFpaLut)
{
    return LEP_SetRadTFpaLut(&_camera_port_descriptor, &radTFpaLut);
}

LEP_RAD_LUT256_T LeptonCamera::GetRadTAuxLut()
{
    LEP_RAD_LUT256_T data;
    LEP_GetRadTAuxLut(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadTAuxLut(LEP_RAD_LUT256_T radTAuxLut)
{
    return LEP_SetRadTAuxLut(&_camera_port_descriptor, &radTAuxLut);
}

LEP_RAD_LUT128_T LeptonCamera::GetRadResponsivityValueLut()
{
    LEP_RAD_LUT128_T data;
    LEP_GetRadResponsivityValueLut(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadResponsivityValueLut(LEP_RAD_LUT128_T radResponsivityValueLut)
{
    return LEP_SetRadResponsivityValueLut(&_camera_port_descriptor, &radResponsivityValueLut);
}

LEP_RAD_KELVIN_T LeptonCamera::GetRadDebugTemp()
{
    LEP_RAD_KELVIN_T data;
    LEP_GetRadDebugTemp(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadDebugTemp(LEP_RAD_KELVIN_T radDebugTemp)
{
    return LEP_SetRadDebugTemp(&_camera_port_descriptor, radDebugTemp);
}

LEP_RAD_FLUX_T LeptonCamera::GetRadDebugFlux()
{
    LEP_RAD_FLUX_T data;
    LEP_GetRadDebugFlux(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadDebugFlux(LEP_RAD_FLUX_T radDebugFlux)
{
    return LEP_SetRadDebugFlux(&_camera_port_descriptor, radDebugFlux);
}

LEP_RAD_ENABLE_E LeptonCamera::GetRadEnableState()
{
    LEP_RAD_ENABLE_E data;
    LEP_GetRadEnableState(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadEnableState(LEP_RAD_ENABLE_E radEnableState)
{
    return LEP_SetRadEnableState(&_camera_port_descriptor, radEnableState);
}

LEP_RAD_GLOBAL_GAIN_T LeptonCamera::GetRadGlobalGain()
{
    LEP_RAD_GLOBAL_GAIN_T data;
    LEP_GetRadGlobalGain(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadGlobalGain(LEP_RAD_GLOBAL_GAIN_T radGlobalGain)
{
    return LEP_SetRadGlobalGain(&_camera_port_descriptor, radGlobalGain);
}

LEP_RAD_GLOBAL_OFFSET_T LeptonCamera::GetRadGlobalOffset()
{
    LEP_RAD_GLOBAL_OFFSET_T data;
    LEP_GetRadGlobalOffset(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadGlobalOffset(LEP_RAD_GLOBAL_OFFSET_T radGlobalOffset)
{
    return LEP_SetRadGlobalOffset(&_camera_port_descriptor, radGlobalOffset);
}

LEP_RAD_TEMPERATURE_UPDATE_E LeptonCamera::GetRadTFpaCtsMode()
{
    LEP_RAD_TEMPERATURE_UPDATE_E data;
    LEP_GetRadTFpaCtsMode(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadTFpaCtsMode(LEP_RAD_TEMPERATURE_UPDATE_E radTFpaCtsMode)
{
    return LEP_SetRadTFpaCtsMode(&_camera_port_descriptor, radTFpaCtsMode);
}

LEP_RAD_TEMPERATURE_UPDATE_E LeptonCamera::GetRadTAuxCtsMode()
{
    LEP_RAD_TEMPERATURE_UPDATE_E data;
    LEP_GetRadTAuxCtsMode(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadTAuxCtsMode(LEP_RAD_TEMPERATURE_UPDATE_E radTAuxCtsMode)
{
    return LEP_SetRadTAuxCtsMode(&_camera_port_descriptor, radTAuxCtsMode);
}

LEP_RAD_TEMPERATURE_COUNTS_T LeptonCamera::GetRadTFpaCts()
{
    LEP_RAD_TEMPERATURE_COUNTS_T data;
    LEP_GetRadTFpaCts(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadTFpaCts(LEP_RAD_TEMPERATURE_COUNTS_T radTFpaCts)
{
    return LEP_SetRadTFpaCts(&_camera_port_descriptor, radTFpaCts);
}

LEP_RAD_TEMPERATURE_COUNTS_T LeptonCamera::GetRadTAuxCts()
{
    LEP_RAD_TEMPERATURE_COUNTS_T data;
    LEP_GetRadTAuxCts(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadTAuxCts(LEP_RAD_TEMPERATURE_COUNTS_T radTAuxCts)
{
    return LEP_SetRadTAuxCts(&_camera_port_descriptor, radTAuxCts);
}

LEP_RAD_LUT128_T LeptonCamera::GetRadTEqShutterLut()
{
    LEP_RAD_LUT128_T data;
    LEP_GetRadTEqShutterLut(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadTEqShutterLut(LEP_RAD_LUT128_T radTEqShutterLut)
{
    return LEP_SetRadTEqShutterLut(&_camera_port_descriptor, &radTEqShutterLut);
}

LEP_RAD_STATUS_E LeptonCamera::GetRadRunStatus()
{
    LEP_RAD_STATUS_E data;
    LEP_GetRadRunStatus(&_camera_port_descriptor, &data);
    return data;
}

LEP_RAD_FLUX_T LeptonCamera::GetRadTEqShutterFlux()
{
    LEP_RAD_FLUX_T data;
    LEP_GetRadTEqShutterFlux(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadTEqShutterFlux(LEP_RAD_FLUX_T radTEqShutterFlux)
{
    return LEP_SetRadTEqShutterFlux(&_camera_port_descriptor, radTEqShutterFlux);
}

LEP_RAD_FLUX_T LeptonCamera::GetRadMffcFlux()
{
    LEP_RAD_FLUX_T data;
    LEP_GetRadMffcFlux(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadMffcFlux(LEP_RAD_FLUX_T radRadMffcFlux)
{
    return LEP_SetRadMffcFlux(&_camera_port_descriptor, radRadMffcFlux);
}

LEP_RAD_MEDIAN_VALUE_T LeptonCamera::GetRadFrameMedianPixelValue()
{
    LEP_RAD_MEDIAN_VALUE_T data;
    LEP_GetRadFrameMedianPixelValue(&_camera_port_descriptor, &data);
    return data;
}

LEP_RAD_SIGNED_LUT128_T LeptonCamera::GetRadMLGLut()
{
    LEP_RAD_SIGNED_LUT128_T data;
    LEP_GetRadMLGLut(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadMLGLut(LEP_RAD_SIGNED_LUT128_T radMLGLut)
{
    return LEP_SetRadMLGLut(&_camera_port_descriptor, &radMLGLut);
}

LEP_RAD_LINEAR_TEMP_CORRECTION_T LeptonCamera::GetRadHousingTcp()
{
    LEP_RAD_LINEAR_TEMP_CORRECTION_T data;
    LEP_GetRadHousingTcp(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadHousingTcp(LEP_RAD_LINEAR_TEMP_CORRECTION_T radHousingTcp)
{
    return LEP_SetRadHousingTcp(&_camera_port_descriptor, radHousingTcp);
}

LEP_RAD_LINEAR_TEMP_CORRECTION_T LeptonCamera::GetRadShutterTcp()
{
    LEP_RAD_LINEAR_TEMP_CORRECTION_T data;
    LEP_GetRadShutterTcp(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadShutterTcp(LEP_RAD_LINEAR_TEMP_CORRECTION_T radShutterTcp)
{
    return LEP_SetRadShutterTcp(&_camera_port_descriptor, radShutterTcp);
}

LEP_RAD_LINEAR_TEMP_CORRECTION_T LeptonCamera::GetRadLensTcp()
{
    LEP_RAD_LINEAR_TEMP_CORRECTION_T data;
    LEP_GetRadLensTcp(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadLensTcp(LEP_RAD_LINEAR_TEMP_CORRECTION_T radLensTcp)
{
    return LEP_SetRadLensTcp(&_camera_port_descriptor, radLensTcp);
}

LEP_RAD_GLOBAL_OFFSET_T LeptonCamera::GetRadPreviousGlobalOffset()
{
    LEP_RAD_GLOBAL_OFFSET_T data;
    LEP_GetRadPreviousGlobalOffset(&_camera_port_descriptor, &data);
    return data;
}

LEP_RAD_GLOBAL_GAIN_T LeptonCamera::GetRadPreviousGlobalGain()
{
    LEP_RAD_GLOBAL_GAIN_T data;
    LEP_GetRadPreviousGlobalGain(&_camera_port_descriptor, &data);
    return data;
}

LEP_RAD_GLOBAL_GAIN_T LeptonCamera::GetGlobalGainFFC()
{
    LEP_RAD_GLOBAL_GAIN_T data;
    LEP_GetGlobalGainFFC(&_camera_port_descriptor, &data);
    return data;
}

LEP_RAD_PARAMETER_SCALE_FACTOR_T LeptonCamera::GetRadCnfScaleFactor()
{
    LEP_RAD_PARAMETER_SCALE_FACTOR_T data;
    LEP_GetRadCnfScaleFactor(&_camera_port_descriptor, &data);
    return data;
}

LEP_RAD_PARAMETER_SCALE_FACTOR_T LeptonCamera::GetRadTnfScaleFactor()
{
    LEP_RAD_PARAMETER_SCALE_FACTOR_T data;
    LEP_GetRadTnfScaleFactor(&_camera_port_descriptor, &data);
    return data;
}

LEP_RAD_PARAMETER_SCALE_FACTOR_T LeptonCamera::GetRadSnfScaleFactor()
{
    LEP_RAD_PARAMETER_SCALE_FACTOR_T data;
    LEP_GetRadSnfScaleFactor(&_camera_port_descriptor, &data);
    return data;
}

LEP_RAD_ARBITRARY_OFFSET_T LeptonCamera::GetRadArbitraryOffset()
{
    LEP_RAD_ARBITRARY_OFFSET_T data;
    LEP_GetRadArbitraryOffset(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadArbitraryOffset(LEP_RAD_ARBITRARY_OFFSET_T arbitraryOffset)
{
    return LEP_SetRadArbitraryOffset(&_camera_port_descriptor, arbitraryOffset);
}

LEP_RAD_FLUX_LINEAR_PARAMS_T LeptonCamera::GetRadFluxLinearParams()
{
    LEP_RAD_FLUX_LINEAR_PARAMS_T data;
    LEP_GetRadFluxLinearParams(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadFluxLinearParams(LEP_RAD_FLUX_LINEAR_PARAMS_T fluxParams)
{
    return LEP_SetRadFluxLinearParams(&_camera_port_descriptor, fluxParams);
}

LEP_RAD_ENABLE_E LeptonCamera::GetRadTLinearEnableState()
{
    LEP_RAD_ENABLE_E data;
    LEP_GetRadTLinearEnableState(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadTLinearEnableState(LEP_RAD_ENABLE_E enableState)
{
    return LEP_SetRadTLinearEnableState(&_camera_port_descriptor, enableState);
}

LEP_RAD_TLINEAR_RESOLUTION_E LeptonCamera::GetRadTLinearResolution()
{
    LEP_RAD_TLINEAR_RESOLUTION_E data;
    LEP_GetRadTLinearResolution(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadTLinearResolution(LEP_RAD_TLINEAR_RESOLUTION_E resolution)
{
    return LEP_SetRadTLinearResolution(&_camera_port_descriptor, resolution);
}

LEP_RAD_ENABLE_E LeptonCamera::GetRadTLinearAutoResolution()
{
    LEP_RAD_ENABLE_E data;
    LEP_GetRadTLinearAutoResolution(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadTLinearAutoResolution(LEP_RAD_ENABLE_E enableState)
{
    return LEP_SetRadTLinearAutoResolution(&_camera_port_descriptor, enableState);
}

LEP_RAD_ROI_T LeptonCamera::GetRadSpotmeterRoi()
{
    LEP_RAD_ROI_T data;
    LEP_GetRadSpotmeterRoi(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadSpotmeterRoi(LEP_RAD_ROI_T spotmeterRoi)
{
    return LEP_SetRadSpotmeterRoi(&_camera_port_descriptor, spotmeterRoi);
}

LEP_RAD_SPOTMETER_OBJ_KELVIN_T LeptonCamera::GetRadSpotmeterObjInKelvinX100()
{
    LEP_RAD_SPOTMETER_OBJ_KELVIN_T data;
    LEP_GetRadSpotmeterObjInKelvinX100(&_camera_port_descriptor, &data);
    return data;
}

LEP_RAD_ARBITRARY_OFFSET_MODE_E LeptonCamera::GetRadArbitraryOffsetMode()
{
    LEP_RAD_ARBITRARY_OFFSET_MODE_E data;
    LEP_GetRadArbitraryOffsetMode(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadArbitraryOffsetMode(LEP_RAD_ARBITRARY_OFFSET_MODE_E arbitraryOffsetMode)
{
    return LEP_SetRadArbitraryOffsetMode(&_camera_port_descriptor, arbitraryOffsetMode);
}

LEP_RAD_ARBITRARY_OFFSET_PARAMS_T LeptonCamera::GetRadArbitraryOffsetParams()
{
    LEP_RAD_ARBITRARY_OFFSET_PARAMS_T data;
    LEP_GetRadArbitraryOffsetParams(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadArbitraryOffsetParams(LEP_RAD_ARBITRARY_OFFSET_PARAMS_T arbitraryOffsetParams)
{
    return LEP_SetRadArbitraryOffsetParams(&_camera_port_descriptor, arbitraryOffsetParams);
}

LEP_RBFO_T LeptonCamera::GetRadInternalRBFOHighGain()
{
    LEP_RBFO_T data;
    LEP_GetRadInternalRBFOHighGain(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadInternalRBFOHighGain(LEP_RBFO_T radRBFO)
{
    return LEP_SetRadInternalRBFOHighGain(&_camera_port_descriptor, &radRBFO);
}

LEP_RBFO_T LeptonCamera::GetRadExternalRBFOHighGain()
{
    LEP_RBFO_T data;
    LEP_GetRadExternalRBFOHighGain(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadExternalRBFOHighGain(LEP_RBFO_T radRBFO)
{
    return LEP_SetRadExternalRBFOHighGain(&_camera_port_descriptor, &radRBFO);
}

LEP_RBFO_T LeptonCamera::GetRadInternalRBFOLowGain()
{
    LEP_RBFO_T data;
    LEP_GetRadInternalRBFOLowGain(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadInternalRBFOLowGain(LEP_RBFO_T radRBFO)
{
    return LEP_SetRadInternalRBFOLowGain(&_camera_port_descriptor, &radRBFO);
}

LEP_RBFO_T LeptonCamera::GetRadExternalRBFOLowGain()
{
    LEP_RBFO_T data;
    LEP_GetRadExternalRBFOLowGain(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadExternalRBFOLowGain(LEP_RBFO_T radRBFO)
{
    return LEP_SetRadExternalRBFOLowGain(&_camera_port_descriptor, &radRBFO);
}

LEP_RAD_RADIO_CAL_VALUES_T LeptonCamera::GetRadRadioCalValues()
{
    LEP_RAD_RADIO_CAL_VALUES_T data;
    LEP_GetRadRadioCalValues(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetRadRadioCalValues(LEP_RAD_RADIO_CAL_VALUES_T radRadioCalValues)
{
    return LEP_SetRadRadioCalValues(&_camera_port_descriptor, radRadioCalValues);
}

/***************************************************************************************************
 *
 *      Lepton SKD Wrapper Functions
 *
 **************************************************************************************************/

LEP_SDK_VERSION_T LeptonCamera::GetSDKVersion()
{
    LEP_SDK_VERSION_T data;
    LEP_GetSDKVersion(&_camera_port_descriptor, &data);
    return data;
}

LEP_SDK_BOOT_STATUS_E LeptonCamera::GetCameraBootStatus()
{
    std::cout << "LeptonCamera::GetCameraBootStatus() was called, but the function is not supported over UVC.\n";
    return LEP_SDK_BOOT_STATUS_E::LEP_BOOT_STATUS_NOT_BOOTED;
    LEP_SDK_BOOT_STATUS_E data;
    LEP_GetCameraBootStatus(&_camera_port_descriptor, &data);
    return data;
}

/***************************************************************************************************
 *
 *      Lepton SKD SYS Wrapper Functions
 *
 **************************************************************************************************/

LEP_RESULT LeptonCamera::RunSysPing()
{
    return LEP_RunSysPing(&_camera_port_descriptor);
}

LEP_STATUS_T LeptonCamera::GetSysStatus()
{
    LEP_STATUS_T data;
    LEP_GetSysStatus(&_camera_port_descriptor, &data);
    return data;
}

LEP_SYS_FLIR_SERIAL_NUMBER_T LeptonCamera::GetSysFlirSerialNumber()
{
    LEP_SYS_FLIR_SERIAL_NUMBER_T data;
    LEP_GetSysFlirSerialNumber(&_camera_port_descriptor, &data);
    return data;
}

LEP_SYS_CUST_SERIAL_NUMBER_T LeptonCamera::GetSysCustSerialNumber()
{
    LEP_SYS_CUST_SERIAL_NUMBER_T data;
    LEP_GetSysCustSerialNumber(&_camera_port_descriptor, &data);
    return data;
}

LEP_SYS_UPTIME_NUMBER_T LeptonCamera::GetSysCameraUpTime()
{
    LEP_SYS_UPTIME_NUMBER_T data;
    LEP_GetSysCameraUpTime(&_camera_port_descriptor, &data);
    return data;
}

LEP_SYS_AUX_TEMPERATURE_CELCIUS_T LeptonCamera::GetSysAuxTemperatureCelsius()
{
    LEP_SYS_AUX_TEMPERATURE_CELCIUS_T data;
    LEP_GetSysAuxTemperatureCelcius(&_camera_port_descriptor, &data);
    return data;
}

LEP_SYS_FPA_TEMPERATURE_CELCIUS_T LeptonCamera::GetSysFpaTemperatureCelsius()
{
    LEP_SYS_FPA_TEMPERATURE_CELCIUS_T data;
    LEP_GetSysFpaTemperatureCelcius(&_camera_port_descriptor, &data);
    return data;
}

LEP_SYS_AUX_TEMPERATURE_KELVIN_T LeptonCamera::GetSysAuxTemperatureKelvin()
{
    LEP_SYS_AUX_TEMPERATURE_KELVIN_T data;
    LEP_GetSysAuxTemperatureKelvin(&_camera_port_descriptor, &data);
    return data;
}

LEP_SYS_AUX_TEMPERATURE_KELVIN_T LeptonCamera::GetSysFpaTemperatureKelvin()
{
    LEP_SYS_AUX_TEMPERATURE_KELVIN_T data;
    LEP_GetSysFpaTemperatureKelvin(&_camera_port_descriptor, &data);
    return data;
}

LEP_SYS_TELEMETRY_ENABLE_STATE_E LeptonCamera::GetSysTelemetryEnableState()
{
    LEP_SYS_TELEMETRY_ENABLE_STATE_E data;
    LEP_GetSysTelemetryEnableState(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetSysTelemetryEnableState(LEP_SYS_TELEMETRY_ENABLE_STATE_E enableState)
{
    return LEP_SetSysTelemetryEnableState(&_camera_port_descriptor, enableState);
}

LEP_SYS_TELEMETRY_LOCATION_E LeptonCamera::GetSysTelemetryLocation()
{
    LEP_SYS_TELEMETRY_LOCATION_E data;
    LEP_GetSysTelemetryLocation(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetSysTelemetryLocation(LEP_SYS_TELEMETRY_LOCATION_E telemetryLocation)
{
    return LEP_SetSysTelemetryLocation(&_camera_port_descriptor, telemetryLocation);
}


LEP_RESULT LeptonCamera::RunSysAverageFrames(LEP_SYS_FRAME_AVERAGE_DIVISOR_E numFrameToAverage)
{
    return LEP_RunSysAverageFrames(&_camera_port_descriptor, numFrameToAverage);
}

LEP_SYS_FRAME_AVERAGE_DIVISOR_E LeptonCamera::GetSysFramesToAverage()
{
    LEP_SYS_FRAME_AVERAGE_DIVISOR_E data;
    LEP_GetSysFramesToAverage(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetSysFramesToAverage(LEP_SYS_FRAME_AVERAGE_DIVISOR_E numFrameToAverage)
{
    return LEP_SetSysFramesToAverage(&_camera_port_descriptor, numFrameToAverage);
}

LEP_SYS_SCENE_STATISTICS_T LeptonCamera::GetSysSceneStatistics()
{
    LEP_SYS_SCENE_STATISTICS_T data;
    LEP_GetSysSceneStatistics(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::RunFrameAverage()
{
    return LEP_RunFrameAverage(&_camera_port_descriptor);
}

LEP_SYS_VIDEO_ROI_T LeptonCamera::GetSysSceneRoi()
{
    LEP_SYS_VIDEO_ROI_T data;
    LEP_GetSysSceneRoi(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetSysSceneRoi(LEP_SYS_VIDEO_ROI_T sceneRoi)
{
    return LEP_SetSysSceneRoi(&_camera_port_descriptor, sceneRoi);
}

LEP_SYS_THERMAL_SHUTDOWN_COUNTS_T LeptonCamera::GetSysThermalShutdownCount()
{
    LEP_SYS_THERMAL_SHUTDOWN_COUNTS_T data;
    LEP_GetSysThermalShutdownCount(&_camera_port_descriptor, &data);
    return data;
}

LEP_SYS_SHUTTER_POSITION_E LeptonCamera::GetSysShutterPosition()
{
    LEP_SYS_SHUTTER_POSITION_E data;
    LEP_GetSysShutterPosition(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetSysShutterPosition(LEP_SYS_SHUTTER_POSITION_E shutterPosition)
{
    return LEP_SetSysShutterPosition(&_camera_port_descriptor, shutterPosition);
}

LEP_SYS_FFC_SHUTTER_MODE_OBJ_T LeptonCamera::GetSysFfcShutterModeObj()
{
    LEP_SYS_FFC_SHUTTER_MODE_OBJ_T data;
    LEP_GetSysFfcShutterModeObj(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetSysFfcShutterModeObj(LEP_SYS_FFC_SHUTTER_MODE_OBJ_T shutterModeObj)
{
    return LEP_SetSysFfcShutterModeObj(&_camera_port_descriptor, shutterModeObj);
}

LEP_SYS_STATUS_E LeptonCamera::GetSysFFCStatus()
{
    LEP_SYS_STATUS_E data;
    LEP_GetSysFFCStatus(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::RunSysFFCNormalization()
{
    return LEP_RunSysFFCNormalization(&_camera_port_descriptor);
}

LEP_SYS_GAIN_MODE_E LeptonCamera::GetSysGainMode()
{
    LEP_SYS_GAIN_MODE_E data;
    LEP_GetSysGainMode(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetSysGainMode(LEP_SYS_GAIN_MODE_E gainMode)
{
    return LEP_SetSysGainMode(&_camera_port_descriptor, gainMode);
}

LEP_SYS_FFC_STATES_E LeptonCamera::GetSysFFCStates()
{
    LEP_SYS_FFC_STATES_E data;
    LEP_GetSysFFCStates(&_camera_port_descriptor, &data);
    return data;
}

LEP_SYS_GAIN_MODE_OBJ_T LeptonCamera::GetSysGainModeObj()
{
    LEP_SYS_GAIN_MODE_OBJ_T data;
    LEP_GetSysGainModeObj(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetSysGainModeObj(LEP_SYS_GAIN_MODE_OBJ_T gainModeObj)
{
    return LEP_SetSysGainModeObj(&_camera_port_descriptor, gainModeObj);
}

LEP_SYS_BORESIGHT_VALUES_T LeptonCamera::GetSysBoresightValues()
{
    LEP_SYS_BORESIGHT_VALUES_T data;
    LEP_GetSysBoresightValues(&_camera_port_descriptor, &data);
    return data;
}

/***************************************************************************************************
 *
 *      Lepton SKD VID Wrapper Functions
 *
 **************************************************************************************************/

LEP_POLARITY_E LeptonCamera::GetVidPolarity()
{
    LEP_POLARITY_E data;
    LEP_GetVidPolarity(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetVidPolarity(LEP_POLARITY_E vidPolarity)
{
    return LEP_SetVidPolarity(&_camera_port_descriptor, vidPolarity);
}

LEP_PCOLOR_LUT_E LeptonCamera::GetVidPcolorLut()
{
    LEP_PCOLOR_LUT_E data;
    LEP_GetVidPcolorLut(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetVidPcolorLut(LEP_PCOLOR_LUT_E vidPcolorLut)
{
    return LEP_SetVidPcolorLut(&_camera_port_descriptor, vidPcolorLut);
}

LEP_PCOLOR_LUT_E LeptonCamera::GetVidLowGainPcolorLut()
{
    LEP_PCOLOR_LUT_E data;
    LEP_GetVidLowGainPcolorLut(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetVidLowGainPcolorLut(LEP_PCOLOR_LUT_E vidPcolorLut)
{
    return LEP_SetVidLowGainPcolorLut(&_camera_port_descriptor, vidPcolorLut);
}

LEP_VID_LUT_BUFFER_T LeptonCamera::GetVidUserLut()
{
    LEP_VID_LUT_BUFFER_T data;
    LEP_GetVidUserLut(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetVidUserLut(LEP_VID_LUT_BUFFER_T vidUserLutBuf)
{
    return LEP_SetVidUserLut(&_camera_port_descriptor, &vidUserLutBuf);
}

LEP_VID_FOCUS_CALC_ENABLE_E LeptonCamera::GetVidFocusCalcEnableState()
{
    LEP_VID_FOCUS_CALC_ENABLE_E data;
    LEP_GetVidFocusCalcEnableState(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetVidFocusCalcEnableState(LEP_VID_FOCUS_CALC_ENABLE_E vidFocusCalcEnableState)
{
    return LEP_SetVidFocusCalcEnableState(&_camera_port_descriptor, vidFocusCalcEnableState);
}

LEP_VID_BORESIGHT_CALC_ENABLE_STATE_E LeptonCamera::GetVidBoresightCalcEnableState()
{
    LEP_VID_BORESIGHT_CALC_ENABLE_STATE_E data;
    LEP_GetVidBoresightCalcEnableState(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetVidBoresightCalcEnableState(LEP_VID_BORESIGHT_CALC_ENABLE_STATE_E boresightCalcEnableState)
{
    return LEP_SetVidBoresightCalcEnableState(&_camera_port_descriptor, boresightCalcEnableState);
}

LEP_VID_BORESIGHT_COORDINATES_T LeptonCamera::GetVidBoresightCoordinates()
{
    LEP_VID_BORESIGHT_COORDINATES_T data;
    LEP_GetVidBoresightCoordinates(&_camera_port_descriptor, &data);
    return data;
}

LEP_VID_TARGET_POSITION_T LeptonCamera::GetVidTargetPosition()
{
    LEP_VID_TARGET_POSITION_T data;
    LEP_GetVidTargetPosition(&_camera_port_descriptor, &data);
    return data;
}

LEP_VID_FOCUS_ROI_T LeptonCamera::GetVidROI()
{
    LEP_VID_FOCUS_ROI_T data;
    LEP_GetVidROI(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetVidROI(LEP_VID_FOCUS_ROI_T vidFocusROI)
{
    return LEP_SetVidROI(&_camera_port_descriptor, vidFocusROI);
}

LEP_VID_FOCUS_METRIC_T LeptonCamera::GetVidFocusMetric()
{
    LEP_VID_FOCUS_METRIC_T data;
    LEP_GetVidFocusMetric(&_camera_port_descriptor, &data);
    return data;
}

LEP_VID_FOCUS_METRIC_THRESHOLD_T LeptonCamera::GetVidFocusMetricThreshold()
{
    LEP_VID_FOCUS_METRIC_THRESHOLD_T data;
    LEP_GetVidFocusMetricThreshold(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetVidFocusMetricThreshold(LEP_VID_FOCUS_METRIC_THRESHOLD_T vidFocusMetricThreshold)
{
    return LEP_SetVidFocusMetricThreshold(&_camera_port_descriptor, vidFocusMetricThreshold);
}

LEP_VID_SBNUC_ENABLE_E LeptonCamera::GetVidSbNucEnableState()
{
    LEP_VID_SBNUC_ENABLE_E data;
    LEP_GetVidSbNucEnableState(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetVidSbNucEnableState(LEP_VID_SBNUC_ENABLE_E vidSbNucEnableState)
{
    return LEP_SetVidSbNucEnableState(&_camera_port_descriptor, vidSbNucEnableState);
}

LEP_VID_VIDEO_OUTPUT_FORMAT_E LeptonCamera::GetVidVideoOutputFormat()
{
    LEP_VID_VIDEO_OUTPUT_FORMAT_E data;
    LEP_GetVidVideoOutputFormat(&_camera_port_descriptor, &data);
    return data;
}

LEP_RESULT LeptonCamera::SetVidVideoOutputFormat(LEP_VID_VIDEO_OUTPUT_FORMAT_E vidVideoOutputFormat)
{
    return LEP_SetVidVideoOutputFormat(&_camera_port_descriptor, vidVideoOutputFormat);
}


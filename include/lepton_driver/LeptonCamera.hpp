
#ifndef LEPTON_DRIVER_LEPTONCAMERA_HPP
#define LEPTON_DRIVER_LEPTONCAMERA_HPP

#include <functional>
#include <cstdint>

#include "libuvc/libuvc.h"
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"

#include <lepton_sdk/LEPTON_AGC.h>
#include "lepton_sdk/LEPTON_OEM.h"
#include "lepton_sdk/LEPTON_RAD.h"
#include "lepton_sdk/LEPTON_SDK.h"
#include "lepton_sdk/LEPTON_SYS.h"
#include "lepton_sdk/LEPTON_VID.h"
#include "lepton_sdk/LEPTON_Types.h"
#include "lepton_driver/UvcManager.hpp"

constexpr uint16_t LEPTON_VID = 0x1e4e;
constexpr uint16_t LEPTON_PID = 0x0100;
constexpr uint16_t LEPTON_RESOLUTION_WIDTH = 160;
constexpr uint16_t LEPTON_RESOLUTION_HEIGHT = 120;

enum LeptonCaptureBits
{
    LEPTON_CAPTURE_8_BIT = UVC_FRAME_FORMAT_UYVY,
    LEPTON_CAPTURE_14_BIT = UVC_FRAME_FORMAT_GRAY16,
};

class LeptonCamera
{
public:
    LeptonCamera(uint16_t port_id, LeptonCaptureBits capture_bits,
                 std::function<void(uvc_frame_t *)> callback = nullptr, bool verbose = false);

    ~LeptonCamera() noexcept;

    /**
     * @brief Changes the capture bits to the provided input
     * @param input LEPTON_CAPTURE_8_BIT or LEPTON_CAPTURE_16_BIT
     * @return True if the number of bits was set and
     * @warning I've had a fair bit of trouble with this function. It stems from libuvc doing whatever it wants.
     * Sometimes when you try to set it to a different number of bits, it doesn't actually set it. Sometimes it can
     * take a long time to complete this function. I've measured 15 seconds for this function to complete in some cases
     * and 0.5 seconds in others. Consider not using this function unless you really need to.
     */
    bool SetCaptureBits(LeptonCaptureBits input);

    /**
     * @brief Gets the latest frame captured by the Lepton camera
     * @return An OpenCV Mat containing the last frame that was captured by the Lepton Camera. An easy way to get the image without using a custom callback.
     */
    cv::Mat GetLastFrame();

    // Many of the SDK functions have little use or should not be used. The more useful functions are documented while the less useful functions are not.
    // For In depth documentation on the Lepton SDK and its 241 functions, see
    // https://lepton.flir.com/wp-content/uploads/2019/02/flir-lepton-software-interface-description-document-303.pdf

    /***************************************************************************************************
     * 
     *      Lepton SKD AGC Wrapper Functions
     *
     **************************************************************************************************/

    /**
     * @brief Gets if AGC is enabled on the device or not
     * @return LEP_AGC_ENABLE or LEP_AGC_DISABLE
     */
    LEP_AGC_ENABLE_E GetAgcEnableState();

    /**
     * @brief Allows enabling or disabling AGC on the device
     * @param agcEnableState LEP_AGC_ENABLE or LEP_AGC_DISABLE
     * @return Result of modifying the AGC state
     */
    LEP_RESULT SetAgcEnableState(LEP_AGC_ENABLE_E agcEnableState);

    /**
     * @brief Gets the current AGC policy of the device
     * @return LEP_AGC_LINEAR or LEP_AGC_HEQ
     */
    LEP_AGC_POLICY_E GetAgcPolicy();

    /**
     * @brief Sets the AGC Policy on the device to Linear or Histogram Equalization (HEQ)
     * @param agcPolicy LEP_AGC_LINEAR or LEP_AGC_HEQ
     * @return Result of modifying the AGC policy state
     */
    LEP_RESULT SetAgcPolicy(LEP_AGC_POLICY_E agcPolicy);

    /**
     * @brief Gets the current Region of Interest (ROI) of AGC on the device
     * @return start and end column and row used to find the Region of interest.
     */
    LEP_AGC_ROI_T GetAgcROI();

     /**
      * @brief Sets the current Region of Interest (ROI) of AGC on the device
      * @param agcROI LEP_AGC_ROI_T that contains the start and end column and row used to find the Region of interest.
      * @return Result of modifying the ROI.
      */
    LEP_RESULT SetAgcROI(LEP_AGC_ROI_T agcROI);

    LEP_AGC_HISTOGRAM_STATISTICS_T GetAgcHistogramStatistics();

    LEP_UINT16 GetAgcLinearHistogramTailSize();

    LEP_RESULT SetAgcLinearHistogramTailSize(LEP_UINT16 agcLinearHistogramTailSize);

    LEP_UINT16 GetAgcLinearHistogramClipPercent();

    LEP_RESULT SetAgcLinearHistogramClipPercent(LEP_UINT16 agcLinearClipPercent);

    LEP_UINT16 GetAgcLinearMaxGain();

    LEP_RESULT SetAgcLinearMaxGain(LEP_UINT16 agcLinearMaxGain);

    LEP_UINT16 GetAgcLinearMidPoint();

    LEP_RESULT SetAgcLinearMidPoint(LEP_UINT16 agcLinearMidPoint);

    LEP_UINT16 GetAgcLinearDampeningFactor();

    LEP_RESULT SetAgcLinearDampeningFactor(LEP_UINT16 agcLinearDampeningFactor);

    /**
     * @brief Gets the Histogram Equalization Dampening Factor used by the device
     * @return 16 bit unsigned integer between 0 and 256
     */
    LEP_UINT16 GetAgcHeqDampingFactor();

    /**
     * @brief Sets the Histogram Equalization Dampening Factor used by the device
     * @param agcHeqDampingFactor 16 bit unsigned integer between 0 and 256
     * @note Default value is 64
     * @return Result of modifying the HEQ Damping factor.
     */
    LEP_RESULT SetAgcHeqDampingFactor(LEP_UINT16 agcHeqDampingFactor);

    LEP_UINT16 GetAgcHeqMaxGain();

    LEP_RESULT SetAgcHeqMaxGain(LEP_UINT16 agcHeqMaxGain);

    /**
     * @brief Gets the maximum number of pixels allowed to accumulate in any given histogram bin.
     * @return 16 bit unsigned integer between 0 and 4800.
     */
    LEP_UINT16 GetAgcHeqClipLimitHigh();

    /**
     * @brief Sets the maximum number of pixels allowed to accumulate in any given histogram bin.
     * @param agcHeqClipLimitHigh 16 bit unsigned integer between 0 and 4800.
     * @note Default value is 4800
     * @return Result of modifying the HEQ Clip limit high.
     */
    LEP_RESULT SetAgcHeqClipLimitHigh(LEP_UINT16 agcHeqClipLimitHigh);

    /**
     * @brief Gets the artificial population that is added to every non-empty histogram bin.
     * @return 16 bit unsigned integer between 0 and 1024.
     */
    LEP_UINT16 GetAgcHeqClipLimitLow();

    /**
     * @brief Sets the artificial population that is added to every non-empty histogram bin.
     * @param agcHeqClipLimitLow 16 bit unsigned integer between 0 and 1024.
     * @note Default value is 512.
     * @return Result of modifying the HEQ Clip limit low.
     */
    LEP_RESULT SetAgcHeqClipLimitLow(LEP_UINT16 agcHeqClipLimitLow);

    LEP_UINT16 GetAgcHeqBinExtension();

    LEP_RESULT SetAgcHeqBinExtension(LEP_UINT16 agcHeqBinExtension);

    LEP_UINT16 GetAgcHeqMidPoint();

    LEP_RESULT SetAgcHeqMidPoint(LEP_UINT16 agcHeqMidPoint);

    /**
     * @brief Gets the maximum number of pixels in a bin that will be interpreted as an empty bin.
     * @return 16 bit unsigned integer between 0 and 16383 (2^14 - 1).
     */
    LEP_AGC_HEQ_EMPTY_COUNT_T GetAgcHeqEmptyCount();

    /**
     * @brief Sets the maximum number of pixels in a bin that will be interpreted as an empty bin.
     * @param emptyCount 16 bit unsigned integer between 0 and 16383 (2^14 - 1).
     * @note Default value is 2.
     * @return Result of modifying the HEQ empty count.
     */
    LEP_RESULT SetAgcHeqEmptyCount(LEP_AGC_HEQ_EMPTY_COUNT_T emptyCount);

    LEP_AGC_HEQ_NORMALIZATION_FACTOR_T GetAgcHeqNormalizationFactor();

    LEP_RESULT SetAgcHeqNormalizationFactor(LEP_AGC_HEQ_NORMALIZATION_FACTOR_T normalizationFactor);

    /**
     * @brief Gets if the Histogram Eqilization output to 8 bits or 14 bits
     * @return LEP_AGC_SCALE_TO_8_BITS or LEP_AGC_SCALE_TO_14_BITS
     */
    LEP_AGC_HEQ_SCALE_FACTOR_E GetAgcHeqScaleFactor();

    /**
     * @brief Sets the Histogram Eqilization output to 8 bits or 14 bits
     * @param scaleFactor LEP_AGC_SCALE_TO_8_BITS or LEP_AGC_SCALE_TO_14_BITS
     * @brief Default value is LEP_AGC_SCALE_TO_8_BITS.
     * @return Result of modifying the HEQ scale factor.
     */
    LEP_RESULT SetAgcHeqScaleFactor(LEP_AGC_HEQ_SCALE_FACTOR_E scaleFactor);

    /**
     * @brief Gets if AGC calculations are enabled or disabled.
     * @return LEP_AGC_ENABLE or LEP_AGC_DISABLE
     */
    LEP_AGC_ENABLE_E GetAgcCalcEnableState();

    /**
     * @brief Sets if AGC calculations are enabled or disabled
     * @param agcCalculationEnableState LEP_AGC_ENABLE or LEP_AGC_DISABLE
     * @note Default value is LEP_AGC_DISABLE
     * @return Result of modifying the ACG calc enable state.
     */
    LEP_RESULT SetAgcCalcEnableState(LEP_AGC_ENABLE_E agcCalculationEnableState);

    /**
     * @brief Gets the Histogram Equalization linear percent.
     * @return 16 bit unsigned integer between 0 and 100.
     */
    LEP_UINT16 GetAgcHeqLinearPercent();

    /**
     * @brief Sets the Histogram Equalization linear percent.
     * @param agcHeqLinearPercent 16 bit unsigned integer between 0 and 100.
     * @note Default value is 20
     * @return Result of modifying the ACG Histogram equalization linear percent.
     */
    LEP_RESULT SetAgcHeqLinearPercent(LEP_UINT16 agcHeqLinearPercent);

    /***************************************************************************************************
     * 
     *      Lepton SKD OEM Wrapper Functions
     *
     *      LEP_RunOemPowerOn will not work as it writes to a register and this is not possible over USB
     *      Maybe fixable?
     *      LEP_RunOemStandby does nothing, just returns LEP_OK.
     *
     **************************************************************************************************/

    LEP_RESULT RunOemPowerDown();

    LEP_RESULT RunOemPowerOn();

    LEP_RESULT RunOemStandby();

    LEP_RESULT RunOemReboot();

    LEP_RESULT RunOemLowPowerMode1();

    LEP_RESULT RunOemLowPowerMode2();

    LEP_RESULT RunOemBit();

    LEP_OEM_MASK_REVISION_T GetOemMaskRevision();

    LEP_OEM_PART_NUMBER_T GetOemFlirPartNumber();

    LEP_OEM_PART_NUMBER_T GetOemCustPartNumber();

    LEP_OEM_SW_VERSION_T GetOemSoftwareVersion();

    LEP_OEM_VIDEO_OUTPUT_ENABLE_E GetOemVideoOutputEnable();

    LEP_RESULT SetOemVideoOutputEnable(LEP_OEM_VIDEO_OUTPUT_ENABLE_E oemVideoOutputEnable);

    LEP_OEM_VIDEO_OUTPUT_FORMAT_E GetOemVideoOutputFormat();

    LEP_RESULT SetOemVideoOutputFormat(LEP_OEM_VIDEO_OUTPUT_FORMAT_E oemVideoOutputFormat);

    LEP_OEM_VIDEO_OUTPUT_SOURCE_E GetOemVideoOutputSource();

    LEP_RESULT SetOemVideoOutputSource(LEP_OEM_VIDEO_OUTPUT_SOURCE_E oemVideoOutputSource);

    LEP_UINT16 GetOemVideoOutputSourceConstant();

    LEP_RESULT SetOemVideoOutputSourceConstant(LEP_UINT16 oemVideoOutputSourceConstant);

    LEP_OEM_VIDEO_OUTPUT_CHANNEL_E GetOemVideoOutputChannel();

    LEP_RESULT SetOemVideoOutputChannel(LEP_OEM_VIDEO_OUTPUT_CHANNEL_E oemVideoOutputChannel);

    LEP_OEM_VIDEO_GAMMA_ENABLE_E GetOemVideoGammaEnable();

    LEP_RESULT SetOemVideoGammaEnable(LEP_OEM_VIDEO_GAMMA_ENABLE_E oemVideoGammaEnable);

    LEP_OEM_STATUS_E GetOemCalStatus();

    LEP_OEM_FFC_NORMALIZATION_TARGET_T GetOemFFCNormalizationTarget();

    LEP_RESULT SetOemFFCNormalizationTarget(LEP_OEM_FFC_NORMALIZATION_TARGET_T ffcTarget);

    LEP_RESULT RunOemFFCNormalization(LEP_OEM_FFC_NORMALIZATION_TARGET_T ffcTarget);

    LEP_OEM_FRAME_AVERAGE_T GetOemFrameMean();

    LEP_OEM_POWER_STATE_E GetOemPowerMode();

    LEP_RESULT SetOemPowerMode(LEP_OEM_POWER_STATE_E powerMode);

    LEP_RESULT RunOemFFC();

    LEP_OEM_GPIO_MODE_E GetOemGpioMode();

    LEP_RESULT SetOemGpioMode(LEP_OEM_GPIO_MODE_E gpioMode);

    LEP_OEM_VSYNC_DELAY_E GetOemGpioVsyncPhaseDelay();

    LEP_RESULT SetOemGpioVsyncPhaseDelay(LEP_OEM_VSYNC_DELAY_E numHsyncLines);

    LEP_OEM_USER_PARAMS_STATE_E GetOemUserDefaultsState();

    LEP_RESULT RunOemUserDefaultsCopyToOtp();

    LEP_RESULT RunOemUserDefaultsRestore();

    LEP_OEM_THERMAL_SHUTDOWN_ENABLE_T GetOemThermalShutdownEnable();

    LEP_RESULT SetOemThermalShutdownEnable(LEP_OEM_THERMAL_SHUTDOWN_ENABLE_T ThermalShutdownEnableState);

    LEP_OEM_SHUTTER_PROFILE_OBJ_T GetOemShutterProfileObj();

    LEP_RESULT SetOemShutterProfileObj(LEP_OEM_SHUTTER_PROFILE_OBJ_T ShutterProfileObj);

    LEP_OEM_BAD_PIXEL_REPLACE_CONTROL_T GetOemBadPixelReplaceControl();

    LEP_RESULT SetOemBadPixelReplaceControl(LEP_OEM_BAD_PIXEL_REPLACE_CONTROL_T BadPixelReplaceControl);

    LEP_OEM_TEMPORAL_FILTER_CONTROL_T GetOemTemporalFilterControl();

    LEP_RESULT SetOemTemporalFilterControl(LEP_OEM_TEMPORAL_FILTER_CONTROL_T TemporalFilterControl);

    LEP_OEM_COLUMN_NOISE_ESTIMATE_CONTROL_T GetOemColumnNoiseEstimateControl();

    LEP_RESULT SetOemColumnNoiseEstimateControl(LEP_OEM_COLUMN_NOISE_ESTIMATE_CONTROL_T ColumnNoiseEstimateControl);

    LEP_OEM_PIXEL_NOISE_SETTINGS_T GetOemPixelNoiseSettings();

    LEP_RESULT SetOemPixelNoiseSettings(LEP_OEM_PIXEL_NOISE_SETTINGS_T PixelNoiseEstimateControl);

    /***************************************************************************************************
     * 
     *      Lepton SKD RAD Wrapper Functions
     *
     **************************************************************************************************/

    LEP_RAD_TS_MODE_E GetRadTShutterMode();

    LEP_RESULT SetRadTShutterMode(LEP_RAD_TS_MODE_E radTShutterMode);

    LEP_RAD_KELVIN_T GetRadTShutter();

    LEP_RESULT SetRadTShutter(LEP_RAD_KELVIN_T radTShutter);

    LEP_RESULT RunRadFFC();

    LEP_RBFO_T GetRadRBFOInternal0();

    LEP_RESULT SetRadRBFOInternal0(LEP_RBFO_T radRBFO);

    LEP_RBFO_T GetRadRBFOExternal0();

    LEP_RESULT SetRadRBFOExternal0(LEP_RBFO_T radRBFO);

    LEP_RAD_RS_T GetRadResponsivityShift();

    LEP_RESULT SetRadResponsivityShift(LEP_RAD_RS_T radResponsivityShift);

    LEP_RAD_FNUMBER_T GetRadFNumber();

    LEP_RESULT SetRadFNumber(LEP_RAD_FNUMBER_T radFNumber);

    LEP_RAD_TAULENS_T GetRadTauLens();

    LEP_RESULT SetRadTauLens(LEP_RAD_TAULENS_T radTauLens);

    LEP_RAD_RADIOMETRY_FILTER_T GetRadRadometryFilter();

    LEP_RESULT SetRadRadometryFilter(LEP_RAD_RADIOMETRY_FILTER_T radRadiometryFilter);

/* Deprecated: Use GetRadTFpaLut */
    LEP_RAD_LUT256_T GetRadTFpaCLut();

/* Deprecated: Use SetRadTFpaLut */
    LEP_RESULT SetRadTFpaCLut(LEP_RAD_LUT256_T radTFpaCLut);

/* Deprecated: Use GetRadTAuxLut */
    LEP_RAD_LUT256_T GetRadTAuxCLut();

/* Deprecated: Use SetRadTAuxLut */
    LEP_RESULT SetRadTAuxCLut(LEP_RAD_LUT256_T radTAuxCLut);

    LEP_RAD_LUT256_T GetRadTFpaLut();

    LEP_RESULT SetRadTFpaLut(LEP_RAD_LUT256_T radTFpaLut);

    LEP_RAD_LUT256_T GetRadTAuxLut();

    LEP_RESULT SetRadTAuxLut(LEP_RAD_LUT256_T radTAuxLut);

    LEP_RAD_LUT128_T GetRadResponsivityValueLut();

    LEP_RESULT SetRadResponsivityValueLut(LEP_RAD_LUT128_T radResponsivityValueLut);

    LEP_RAD_KELVIN_T GetRadDebugTemp();

    LEP_RESULT SetRadDebugTemp(LEP_RAD_KELVIN_T radDebugTemp);

    LEP_RAD_FLUX_T GetRadDebugFlux();

    LEP_RESULT SetRadDebugFlux(LEP_RAD_FLUX_T radDebugFlux);

    LEP_RAD_ENABLE_E GetRadEnableState();

    LEP_RESULT SetRadEnableState(LEP_RAD_ENABLE_E radEnableState);

    LEP_RAD_GLOBAL_GAIN_T GetRadGlobalGain();

    LEP_RESULT SetRadGlobalGain(LEP_RAD_GLOBAL_GAIN_T radGlobalGain);

    LEP_RAD_GLOBAL_OFFSET_T GetRadGlobalOffset();

    LEP_RESULT SetRadGlobalOffset(LEP_RAD_GLOBAL_OFFSET_T radGlobalOffset);

    LEP_RAD_TEMPERATURE_UPDATE_E GetRadTFpaCtsMode();

    LEP_RESULT SetRadTFpaCtsMode(LEP_RAD_TEMPERATURE_UPDATE_E radTFpaCtsMode);

    LEP_RAD_TEMPERATURE_UPDATE_E GetRadTAuxCtsMode();

    LEP_RESULT SetRadTAuxCtsMode(LEP_RAD_TEMPERATURE_UPDATE_E radTAuxCtsMode);

    LEP_RAD_TEMPERATURE_COUNTS_T GetRadTFpaCts();

    LEP_RESULT SetRadTFpaCts(LEP_RAD_TEMPERATURE_COUNTS_T radTFpaCts);

    LEP_RAD_TEMPERATURE_COUNTS_T GetRadTAuxCts();

    LEP_RESULT SetRadTAuxCts(LEP_RAD_TEMPERATURE_COUNTS_T radTAuxCts);

    LEP_RAD_LUT128_T GetRadTEqShutterLut();

    LEP_RESULT SetRadTEqShutterLut(LEP_RAD_LUT128_T radTEqShutterLut);

    LEP_RAD_STATUS_E GetRadRunStatus();

    LEP_RAD_FLUX_T GetRadTEqShutterFlux();

    LEP_RESULT SetRadTEqShutterFlux(LEP_RAD_FLUX_T radTEqShutterFlux);

    LEP_RAD_FLUX_T GetRadMffcFlux();

    LEP_RESULT SetRadMffcFlux(LEP_RAD_FLUX_T radRadMffcFlux);

    LEP_RAD_MEDIAN_VALUE_T GetRadFrameMedianPixelValue();

    LEP_RAD_SIGNED_LUT128_T GetRadMLGLut();

    LEP_RESULT SetRadMLGLut(LEP_RAD_SIGNED_LUT128_T radMLGLut);

    LEP_RAD_LINEAR_TEMP_CORRECTION_T GetRadHousingTcp();

    LEP_RESULT SetRadHousingTcp(LEP_RAD_LINEAR_TEMP_CORRECTION_T radHousingTcp);

    LEP_RAD_LINEAR_TEMP_CORRECTION_T GetRadShutterTcp();

    LEP_RESULT SetRadShutterTcp(LEP_RAD_LINEAR_TEMP_CORRECTION_T radShutterTcp);

    LEP_RAD_LINEAR_TEMP_CORRECTION_T GetRadLensTcp();

    LEP_RESULT SetRadLensTcp(LEP_RAD_LINEAR_TEMP_CORRECTION_T radLensTcp);

    LEP_RAD_GLOBAL_OFFSET_T GetRadPreviousGlobalOffset();

    LEP_RAD_GLOBAL_GAIN_T GetRadPreviousGlobalGain();

    LEP_RAD_GLOBAL_GAIN_T GetGlobalGainFFC();

    LEP_RAD_PARAMETER_SCALE_FACTOR_T GetRadCnfScaleFactor();

    LEP_RAD_PARAMETER_SCALE_FACTOR_T GetRadTnfScaleFactor();

    LEP_RAD_PARAMETER_SCALE_FACTOR_T GetRadSnfScaleFactor();

    LEP_RAD_ARBITRARY_OFFSET_T GetRadArbitraryOffset();

    LEP_RESULT SetRadArbitraryOffset(LEP_RAD_ARBITRARY_OFFSET_T arbitraryOffset);

    LEP_RAD_FLUX_LINEAR_PARAMS_T GetRadFluxLinearParams();

    LEP_RESULT SetRadFluxLinearParams(LEP_RAD_FLUX_LINEAR_PARAMS_T fluxParams);

    LEP_RAD_ENABLE_E GetRadTLinearEnableState();

    LEP_RESULT SetRadTLinearEnableState(LEP_RAD_ENABLE_E enableState);

    LEP_RAD_TLINEAR_RESOLUTION_E GetRadTLinearResolution();

    LEP_RESULT SetRadTLinearResolution(LEP_RAD_TLINEAR_RESOLUTION_E resolution);

    LEP_RAD_ENABLE_E GetRadTLinearAutoResolution();

    LEP_RESULT SetRadTLinearAutoResolution(LEP_RAD_ENABLE_E enableState);

    LEP_RAD_ROI_T GetRadSpotmeterRoi();

    LEP_RESULT SetRadSpotmeterRoi(LEP_RAD_ROI_T spotmeterRoi);

    LEP_RAD_SPOTMETER_OBJ_KELVIN_T GetRadSpotmeterObjInKelvinX100();

    LEP_RAD_ARBITRARY_OFFSET_MODE_E GetRadArbitraryOffsetMode();

    LEP_RESULT SetRadArbitraryOffsetMode(LEP_RAD_ARBITRARY_OFFSET_MODE_E arbitraryOffsetMode);

    LEP_RAD_ARBITRARY_OFFSET_PARAMS_T GetRadArbitraryOffsetParams();

    LEP_RESULT SetRadArbitraryOffsetParams(LEP_RAD_ARBITRARY_OFFSET_PARAMS_T arbitraryOffsetParams);

    LEP_RBFO_T GetRadInternalRBFOHighGain();

    LEP_RESULT SetRadInternalRBFOHighGain(LEP_RBFO_T radRBFO);

    LEP_RBFO_T GetRadExternalRBFOHighGain();

    LEP_RESULT SetRadExternalRBFOHighGain(LEP_RBFO_T radRBFO);

    LEP_RBFO_T GetRadInternalRBFOLowGain();

    LEP_RESULT SetRadInternalRBFOLowGain(LEP_RBFO_T radRBFO);

    LEP_RBFO_T GetRadExternalRBFOLowGain();

    LEP_RESULT SetRadExternalRBFOLowGain(LEP_RBFO_T radRBFO);

    LEP_RAD_RADIO_CAL_VALUES_T GetRadRadioCalValues();

    LEP_RESULT SetRadRadioCalValues(LEP_RAD_RADIO_CAL_VALUES_T radRadioCalValues);

    /***************************************************************************************************
     * 
     *      Lepton SKD Wrapper Functions
     *
     **************************************************************************************************/

    LEP_SDK_VERSION_T GetSDKVersion();

    LEP_SDK_BOOT_STATUS_E GetCameraBootStatus();

    /***************************************************************************************************
     * 
     *      Lepton SKD SYS Wrapper Functions
     *
     **************************************************************************************************/

    /**
     * @brief Sends a Ping command to the camera.
     * @return LEP_OK if the command was recieved by the camera.
     */
    LEP_RESULT RunSysPing();

    /**
     * @brief Gets the status of the camera.
     * @return ELP_SYSTEM_READY if the camera is ready, LEP_SYSTEM_INITIALIZING if the camera is initializing,
     * LEP_SYSTEM_IN_LOW_POWER_MODE if the camera is in low power mode, LEP_SYSTEM_GOING_INTO_STANDBY if the camera is
     * going into standby mode, and LEP_SYSTEM_FLAT_FIELD_IN_PROGRESS if the camera is performing FFC.
     */
    LEP_STATUS_T GetSysStatus();

    /**
     * @brief Gets the FLIR serial number of the Lepton camera
     * @return A uint64_t representing the camera's serial number.
     */
    LEP_SYS_FLIR_SERIAL_NUMBER_T GetSysFlirSerialNumber();

    /**
     * @brief Gets the Customer Serial Number as a 32 byte string.
     * @return 32 char string containing the Customer Serial Number.
     * @note Deprecated: Use GetSysCustSN instead.
     */
    LEP_SYS_CUST_SERIAL_NUMBER_T GetSysCustSerialNumber();

    /**
     * @brief Gets the uptime of the camera in milliseconds.
     * @return A uint32_t containing the uptime of the camera.
     * @note As a 32 bit integer can only store up to 4294967295, if more than that many milliseconds have elapsed the
     * uptime will rollover. This is 1193 hours.
     */
    LEP_SYS_UPTIME_NUMBER_T GetSysCameraUpTime();

    /**
     * @brief Gets the temperature of the AUX(?) of the lepton camera in Celsius.
     * @return A uint16_t representing the temperature of the AUX of the camera.
     * @note The SDK misspells Celsius as Celcius, which can be seen in the return type of this function. This function
     * uses the correct spelling.
     */
    LEP_SYS_AUX_TEMPERATURE_CELCIUS_T GetSysAuxTemperatureCelsius();
    /**
     * @brief Gets the temperature of the focal plane array (FPA) of the lepton camera in Celsius.
     * @return A uint16_t representing the temperature of the AUX of the camera.
     * @note The SDK misspells Celsius as Celcius, which can be seen in the return type of this function. This function
     * uses the correct spelling.
     */
    LEP_SYS_FPA_TEMPERATURE_CELCIUS_T GetSysFpaTemperatureCelsius();

    /**
     * @brief Gets the temperature of the AUX(?) of the lepton camera in Kelvin.
     * @return A uint16_t representing the temperature of the AUX of the camera.
     */
    LEP_SYS_AUX_TEMPERATURE_KELVIN_T GetSysAuxTemperatureKelvin();

    /**
     * @brief Gets the temperature of the focal plane array (FPA) of the lepton camera in Kelvin.
     * @return A uint16_T representing the temperature of the FPA of the camera.
     */
    LEP_SYS_AUX_TEMPERATURE_KELVIN_T GetSysFpaTemperatureKelvin();

    /**
     * @brief Gets the Telemetry Enabled State.
     * @return LEP_TELEMETRY_DISABLED or LEP_TELEMETRY_ENABLED.
     */
    LEP_SYS_TELEMETRY_ENABLE_STATE_E GetSysTelemetryEnableState();

    /**
     * @brief Sets the Telemetry Enable State.
     * @param enableState LEP_TELEMETRY_DISABLED or LEP_TELEMETRY_ENABLED.
     * @note Fefault value is LEP_TELEMETRY_DISABLED
     * @return Result of modifying the Telemetry Enable state.
     */
    LEP_RESULT SetSysTelemetryEnableState(LEP_SYS_TELEMETRY_ENABLE_STATE_E enableState);

    /**
     * @brief Gets the Telemetry Location.
     * @return LEP_TELEMETRY_LOCATION_HEADER or LEP_TELEMETRY_LOCATION_FOOTER
     */
    LEP_SYS_TELEMETRY_LOCATION_E GetSysTelemetryLocation();

    /**
     * @brief Sets the Telemetry Location.
     * @param telemetryLocation LEP_TELEMETRY_LOCATION_HEADER or LEP_TELEMETRY_LOCATION_FOOTER
     * @note Default value is LEP_TELEMETRY_LOCATION_FOOTER
     * @return Result of modifying Telemetry Location.
     */
    LEP_RESULT SetSysTelemetryLocation(LEP_SYS_TELEMETRY_LOCATION_E telemetryLocation);


    /**
     * @brief Sets the Average frames to the provided numFrameToAverage and updates the frames.
     * @param numFrameToAverage Number of frames that will be used for averages.
     * @return Result of setting and running the Average Frames command.
     * @note Compatibility: Lepton 1.5, 1.6, 2.0, 2.5. Lepton 3.0 and 3.5 only use LEP_SYS_FA_DIV_8
     */
    LEP_RESULT RunSysAverageFrames(LEP_SYS_FRAME_AVERAGE_DIVISOR_E numFrameToAverage);

    /**
     * @brief Gets the number of frames used to average.
     * @return LEP_SYS_FA_DIV_1, LEP_SYS_FA_DIV_2, LEP_SYS_FA_DIV_4, LEP_SYS_FA_DIV_8, LEP_SYS_FA_DIV_16, LEP_SYS_FA_DIV_32, or LEP_SYS_FA_DIV_64.
     * @note Compatibility: Lepton 1.5, 1.6, 2.0, 2.5. Lepton 3.0 and 3.5 only use LEP_SYS_FA_DIV_8
     */
    LEP_SYS_FRAME_AVERAGE_DIVISOR_E GetSysFramesToAverage();

    /**
     * @brief Sets the number of frames used to average.
     * @param numFrameToAverage LEP_SYS_FA_DIV_1, LEP_SYS_FA_DIV_2, LEP_SYS_FA_DIV_4, LEP_SYS_FA_DIV_8, LEP_SYS_FA_DIV_16, LEP_SYS_FA_DIV_32, or LEP_SYS_FA_DIV_64.
     * @return Result of modifying the Average Frames
     * @note Compatibility: Lepton 1.5, 1.6, 2.0, 2.5. Lepton 3.0 and 3.5 only use LEP_SYS_FA_DIV_8
     */
    LEP_RESULT SetSysFramesToAverage(LEP_SYS_FRAME_AVERAGE_DIVISOR_E numFrameToAverage);

    /**
     * @brief Gets statistics of the current scene captured by the camera.
     * @return Mean intensity, max intensity, min intensity, and number of pixels.
     */
    LEP_SYS_SCENE_STATISTICS_T GetSysSceneStatistics();

    /**
     * @brief Executes the Average Frames command.
     * @param numFrameToAverage Number of frames that will be used for averages.
     * @return Result of running the Average Frames command.
     * @note Compatibility: Lepton 1.5, 1.6, 2.0, 2.5. Lepton 3.0 and 3.5 only use LEP_SYS_FA_DIV_8
     */
    LEP_RESULT RunFrameAverage();

    /**
     * @brief Gets the region of interest (ROI) used to calculate scene statistics.
     * @return The start column, end column, start row, and end row used to find the ROI.
     */
    LEP_SYS_VIDEO_ROI_T GetSysSceneRoi();

    /**
     * @brief Sets the region of interest (ROI) used to calculate the scene statisitics.
     * @param sceneRoi The start column, end column, start row, and end row used to find the ROI.
     * @return Result of modifying the ROI.
     */
    LEP_RESULT SetSysSceneRoi(LEP_SYS_VIDEO_ROI_T sceneRoi);

    /**
     * @brief Gets the number of frames remaining before the camera shuts down from exceeding the high temperature threshold.
     * @return A uint16_t containing the number of frames before thermal shutdown.
     */
    LEP_SYS_THERMAL_SHUTDOWN_COUNTS_T GetSysThermalShutdownCount();

    /**
     * @brief Gets the current position of the shutter of the camera.
     * @return LEP_SYS_SHUTTER_POSITION_UNKNOWN, LEP_SYS_SHUTTER_POSITION_IDLE, LEP_SYS_SHUTTER_POSITION_OPEN,
     * LEP_SYS_SHUTTER_POSITION_CLOSED, or LEP_SYS_SHUTTER_POSITION_BRAKE_ON.
     * @note If the camera has no shutter, LEP_SYS_SHUTTER_POSITION_UNKNOWN is returned.
     */
    LEP_SYS_SHUTTER_POSITION_E GetSysShutterPosition();

    /**
     * @brief Sets the position of the shutter of the camera.
     * @param shutterPosition LEP_SYS_SHUTTER_POSITION_UNKNOWN, LEP_SYS_SHUTTER_POSITION_IDLE,
     * LEP_SYS_SHUTTER_POSITION_OPEN, LEP_SYS_SHUTTER_POSITION_CLOSED, or LEP_SYS_SHUTTER_POSITION_BRAKE_ON.
     * @return Result of modifying the shutter position
     */
    LEP_RESULT SetSysShutterPosition(LEP_SYS_SHUTTER_POSITION_E shutterPosition);

    /**
     * @brief Gets the controls used by the shutter during FFC.
     * @return The shutter mode during FFC, if the shutter is controllable during FFC, if the video freezes during FFC,
     * and if FFC is desired.
     */
    LEP_SYS_FFC_SHUTTER_MODE_OBJ_T GetSysFfcShutterModeObj();

    /**
     * @brief Sets the controls used by the shutter during FFC.
     * @param shutterModeObj The shutter mode during FFC, if the shutter is controllable during FFC, if the video
     * freezes during FFC, and if FFC is desired.
     * @return Result of modifying the shutter control mode.
     */
    LEP_RESULT SetSysFfcShutterModeObj(LEP_SYS_FFC_SHUTTER_MODE_OBJ_T shutterModeObj);

    /**
     * @brief Performs the Flat-Field Correction (FFC) on the device.
     * @return The result of performing FFC.
     */
    LEP_RESULT RunSysFFCNormalization();

    /**
     * @brief Gets the status of Flat-Field Correction (FFC)
     * @return LEP_SYS_STATUS_WRITE_ERROR, LEP_SYS_STATUS_ERROR, LEP_SYS_STATUS_READY, LEP_SYS_STATUS_BUSY, or
     * LEP_SYS_FRAME_AVERAGE_COLLECTING_FRAMES.
     */
    LEP_SYS_STATUS_E GetSysFFCStatus();

    /**
     * @brief Gets the gain state of the camera.
     * @return LEP_SYS_GAIN_MODE_HIGH, LEP_SYS_GAIN_MODE_LOW, or LEP_SYS_GAIN_MODE_AUTO.
     */
    LEP_SYS_GAIN_MODE_E GetSysGainMode();

    /**
     * @brief Sets the gain state of the camera.
     * @param gainMode LEP_SYS_GAIN_MODE_HIGH, LEP_SYS_GAIN_MODE_LOW, or LEP_SYS_GAIN_MODE_AUTO.
     * @note Default setting is LEP_SYS_GAIN_MODE_HIGH
     * @return The result of setting the gain state.
     * @note Compatibility: Lepton 2.5, 3.5
     */
    LEP_RESULT SetSysGainMode(LEP_SYS_GAIN_MODE_E gainMode);

    /**
     * @brief Gets the Flat-Field Correction (FFC) state.
     * @return LEP_SYS_FFC_NEVER_COMMANDED, LEP_SYS_FFC_IMMINENT,  LEP_SYS_FFC_IN_PROCESS,  or LEP_SYS_FFC_DONE.
     */
    LEP_SYS_FFC_STATES_E GetSysFFCStates();

    /**
     * @brief Gets gain parameters used by the camera.
     * @return Parameters used by the gain of the camera.
     */
    LEP_SYS_GAIN_MODE_OBJ_T GetSysGainModeObj();

    /**
     * @brief Sets gain parameters used by the camera.
     * @param gainModeObj Parameters used by the gain of the camera.
     * @return The result of setting the gain parameters.
     */
    LEP_RESULT SetSysGainModeObj(LEP_SYS_GAIN_MODE_OBJ_T gainModeObj);

    LEP_SYS_BORESIGHT_VALUES_T GetSysBoresightValues();

    /***************************************************************************************************
     * 
     *      Lepton SKD VID Wrapper Functions
     *
     **************************************************************************************************/

    LEP_POLARITY_E GetVidPolarity();

    LEP_RESULT SetVidPolarity(LEP_POLARITY_E vidPolarity);

    /**
     * @brief Gets the pseudo-color LUT used when reading 24 bit RGB.
     * @return LEP_VID_WHEEL6_LUT, LEP_VID_FUSION_LUT, LEP_VID_RAI, LEP_VID_GLOBOW_LUT, LEP_VID_SEPIA_LUT,
     * LEP_VID_COLOR_LUT, LEP_VID_ICE_FIRE_LUT, LEP_VID_RAIN_LUT, LEP_VID_USER_LUT, or LEP_VID_END_PCOLOR_LUT.
     */
    LEP_PCOLOR_LUT_E GetVidPcolorLut();

    /**
     * @brief Sets the pseudo-color LUT used when reading 24 bit RBG.
     * @param vidPcolorLut LEP_VID_WHEEL6_LUT, LEP_VID_FUSION_LUT, LEP_VID_RAI, LEP_VID_GLOBOW_LUT, LEP_VID_SEPIA_LUT,
     * LEP_VID_COLOR_LUT, LEP_VID_ICE_FIRE_LUT, LEP_VID_RAIN_LUT, LEP_VID_USER_LUT, or LEP_VID_END_PCOLOR_LUT.
     * @return The result of setting the pseudo-color LUT.
     */
    LEP_RESULT SetVidPcolorLut(LEP_PCOLOR_LUT_E vidPcolorLut);

    /**
     * @brief Gets the pseudo-color look-up table used when the camera is in low gain mode.
     * @return LEP_VID_WHEEL6_LUT, LEP_VID_FUSION_LUT, LEP_VID_RAINBOW_LUT, LEP_VID_GLOBOW_LUT, LEP_VID_SEPIA_LUT,
     * LEP_VID_COLOR_LUT, LEP_VID_ICE_FIRE_LUT, LEP_VID_RAIN_LUT, or LEP_VID_USER_LUT.
     */
    LEP_PCOLOR_LUT_E GetVidLowGainPcolorLut();

    /**
     * @brief Sets the pseudo-colo look-up table used when the camera is in low gain mode.
     * @param vidPcolorLut LEP_VID_WHEEL6_LUT, LEP_VID_FUSION_LUT, LEP_VID_RAINBOW_LUT, LEP_VID_GLOBOW_LUT,
     * LEP_VID_SEPIA_LUT, LEP_VID_COLOR_LUT, LEP_VID_ICE_FIRE_LUT, LEP_VID_RAIN_LUT, or LEP_VID_USER_LUT.
     * @return The result of setting the pseudo-color look-up table used in low gain mode.
     */
    LEP_RESULT SetVidLowGainPcolorLut(LEP_PCOLOR_LUT_E vidPcolorLut);

    /**
     * @brief Gets a the custom pseudo-color LUT used when reading 24 bit RBG.
     * @return LEP_VID_LUT_PIXEL_T containing red, green, blue, and reserved.
     */
    LEP_VID_LUT_BUFFER_T GetVidUserLut();

    /**
     * @brief Sets a the custom pseudo-color LUT used when reading 24 bit RBG.
     * @param vidUserLutBufPtr LEP_VID_LUT_PIXEL_T containing red, green, blue, and reserved.
     * @return The result of setting the custom pseudo-color LUT.
     */
    LEP_RESULT SetVidUserLut(LEP_VID_LUT_BUFFER_T vidUserLutBufPtr);

    /**
     * @brief Gets if the camera can calculate a video scene focus metric.
     * @return LEP_VID_FOCUS_CALC_DISABLE or LEP_VID_FOCUS_CALC_ENABLE
     */
    LEP_VID_FOCUS_CALC_ENABLE_E GetVidFocusCalcEnableState();

    /**
     * @brief Sets if the camera can calculate a video scene focus metric.
     * @param vidFocusCalcEnableState LEP_VID_FOCUS_CALC_DISABLE or LEP_VID_FOCUS_CALC_ENABLE
     * @return The result of enabling or disabling the video scene focus metric.
     * @note If the video scene focus metric is enabled, then AGC must be disabled.
     */
    LEP_RESULT SetVidFocusCalcEnableState(LEP_VID_FOCUS_CALC_ENABLE_E vidFocusCalcEnableState);

    LEP_VID_BORESIGHT_CALC_ENABLE_STATE_E GetVidBoresightCalcEnableState();

    LEP_RESULT SetVidBoresightCalcEnableState(LEP_VID_BORESIGHT_CALC_ENABLE_STATE_E boresightCalcEnableState);

    LEP_VID_BORESIGHT_COORDINATES_T GetVidBoresightCoordinates();

    LEP_VID_TARGET_POSITION_T GetVidTargetPosition();

    /**
     * @brief Gets the Region of Interest used for calculating a video scene focus metric.
     * @return LEP_VID_FOCUS_ROI_T containing the start column, end column, start row, and end row of the ROI.
     */
    LEP_VID_FOCUS_ROI_T GetVidROI();

    /**
     * @brief Sets the Region of Interest used for calculating a video scene focus metric.
     * @param vidFocusROI LEP_VID_FOCUS_ROI_T containing the start column, end column, start row, and end row of the ROI.
     * @return The result of setting the region of interest for calculating a video scene focus metric.
     */
    LEP_RESULT SetVidROI(LEP_VID_FOCUS_ROI_T vidFocusROI);

    LEP_VID_FOCUS_METRIC_T GetVidFocusMetric();

    /**
     * @brief Gets the focus metric threshold.
     * @return A uint32_t representing the focus metric threshold.
     */
    LEP_VID_FOCUS_METRIC_THRESHOLD_T GetVidFocusMetricThreshold();

    /**
     * @brief Sets the focus metric threshold.
     * @param vidFocusMetricThreshold A uint32_t representing the focus metric threshold.
     * @note Default value is 30
     * @return The result of setting the focus metric threshold.
     */
    LEP_RESULT SetVidFocusMetricThreshold(LEP_VID_FOCUS_METRIC_THRESHOLD_T vidFocusMetricThreshold);

    LEP_VID_SBNUC_ENABLE_E GetVidSbNucEnableState();

    LEP_RESULT SetVidSbNucEnableState(LEP_VID_SBNUC_ENABLE_E vidSbNucEnableState);

    /**
     * @brief Gets the format of captured images.
     * @return LEP_VID_VIDEO_OUTPUT_FORMAT_RGB888, or LEP_VID_VIDEO_OUTPUT_FORMAT_RAW14.
     * @note Other values exist but they are not supported
     */
    LEP_VID_VIDEO_OUTPUT_FORMAT_E GetVidVideoOutputFormat();

    LEP_RESULT SetVidVideoOutputFormat(LEP_VID_VIDEO_OUTPUT_FORMAT_E vidVideoOutputFormat);

private:
    /**
     * @brief Port descriptor used by the Lepton SDK
     */
    LEP_CAMERA_PORT_DESC_T _camera_port_descriptor;

    /**
     * Manages inputs and outputs from libuvc
     */
    UvcManager _uvc_manager;

};

#endif //LEPTON_DRIVER_LEPTONCAMERA_HPP

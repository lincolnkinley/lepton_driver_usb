
#include <cstdint>
#include <chrono>
#include <thread>
#include <functional>
#include <mutex>

#include "opencv2/core.hpp"
#include "libuvc/libuvc.h"

#include "lepton_driver/UvcManager.hpp"
#include "lepton_sdk/uvc_sdk.h"
#include "lepton_sdk/LEPTON_SDK.h"

void uvc_callback(uvc_frame_t *frame, void *ptr)
{
    UvcManager *_this = static_cast<UvcManager *>(ptr);

    if (frame->height != _this->_resolution_height || frame->width != _this->_resolution_width)
    {
        printf("Read image does not match expected resolution\n");
    }
    // Convert uvc_frame_t to OpenCV Mat
    cv::Mat cv_frame(frame->height, frame->width, CV_16U, frame->data);

    if (_this->CaptureCallback != nullptr)
    {
        _this->CaptureCallback(frame);
    }
    _this->SetLastFrame(cv_frame);
}

UvcManager::UvcManager(uint16_t device_vid, uint16_t device_pid, uint16_t resolution_width, uint16_t resolution_height,
                       uvc_frame_format uvc_format, std::function<void(uvc_frame_t *)> callback, bool verbose)
{
    _device_vid = device_vid;
    _device_pid = device_pid;
    _resolution_width = resolution_width;
    _resolution_height = resolution_height;
    _uvc_format = uvc_format;
    CaptureCallback = std::move(callback);
    _verbose = verbose;

    // Initialize UVC
    bool success = InitUvc();
    if (success == false)
    {
        throw;
    }

    // Find the device
    success = FindDevice();
    if (success == false)
    {
        Cleanup();
        throw;
    }

    // Open the device
    success = OpenDevice();
    if (success == false)
    {
        Cleanup();
        throw;
    }

    // Set the format of the device
    success = FormatDevice();
    if (success == false)
    {
        Cleanup();
        throw;
    }

    // Start streaming images from the device
    success = StartStream();
    if (success == false)
    {
        Cleanup();
        throw;
    }
}

UvcManager::~UvcManager()
{
    Cleanup();
}

void UvcManager::StopStream()
{
    uvc_stop_streaming(_devh);
}

bool UvcManager::StartStream()
{
    uvc_error_t res = uvc_start_streaming(_devh, &_ctrl, uvc_callback, this, 0);

    if (res < 0)
    {
        if (_verbose == true)
        {
            printf("StartStream() failed: %d\n", res);
        }
        uvc_close(_devh);

        return false;
    }
    return true;
}

cv::Mat UvcManager::GetLastFrame()
{
    _last_frame_lock.lock();
    cv::Mat ret = _last_frame;
    _last_frame_lock.unlock();
    return ret;
}

void UvcManager::SetLastFrame(cv::Mat input_frame)
{
    _last_frame_lock.lock();
    _last_frame = input_frame;
    _last_frame_lock.unlock();
}

bool UvcManager::SetVideoFormat(uint16_t resolution_width, uint16_t resolution_height, uvc_frame_format uvc_format)
{
    StopStream();

    // WARNING!! The sleep below works on my computer, but I'm not sure about its portability. There seems to be no way to check if a steam is actually stopped.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    uvc_frame_format original_format = _uvc_format;
    uint16_t original_resolution_width = _resolution_width;
    uint16_t original_resolution_height = _resolution_height;

    _uvc_format = uvc_format;
    _resolution_width = resolution_width;
    _resolution_height = resolution_height;
    bool success = FormatDevice();
    if (success == false)
    {
        _uvc_format = original_format;
        _resolution_width = original_resolution_width;
        _resolution_height = original_resolution_height;

        return false;
    }
    success = StartStream();
    if (success == false)
    {
        return false;
    }
    return true;
}

bool UvcManager::InitUvc()
{
    uvc_error_t res = uvc_init(&_ctx, NULL);

    if (res < 0)
    {
        if (_verbose == true)
        {
            printf("InitUvc() failed: %d\n", res);
        }
        return false;
    }
    return true;
}

bool UvcManager::FindDevice()
{
    uvc_error_t res = uvc_find_device(_ctx, &_dev, _device_vid, _device_pid, NULL);

    if (res < 0)
    {
        if (_verbose == true)
        {
            printf("FindDevice() failed: %d\nMake sure that the Lepton camera is plugged in!", res);
        }
        return false;
    }
    return true;
}

bool UvcManager::OpenDevice()
{
    uvc_error_t res = uvc_open(_dev, &_devh);

    if (res < 0)
    {
        if (_verbose == true)
        {
            printf("OpenDevice() failed: %d\nIf you're seeing this, it's likely because the lepton drivers do not have access to the device. You can set this program to run with root privileges, or special privileges are not required to read the VID and PID.\n", res);
            // If you are on linux, run this command
            // find /dev/bus/usb/*/* -exec ls -l {} \;
            // non-privileged users will need write permissions on these files to be able to read the lepton camera.
        }

        uvc_unref_device(_dev);
        _dev = NULL;
        return false;
    }
    return true;
}

bool UvcManager::FormatDevice()
{
    uvc_error_t res = uvc_get_stream_ctrl_format_size(
            _devh, &_ctrl,
            _uvc_format,
            _resolution_width, _resolution_height, 0);

    if (res < 0)
    {
        if (_verbose == true)
        {
            printf("FormatDevice() failed: %d\n", res);
        }
        return false;
    }
    return true;
}

void UvcManager::Cleanup()
{
    if (_devh != NULL)
    {
        StopStream();
        if (_verbose == true)
        {
            printf("Streaming complete.\n");
        }

        uvc_close(_devh);
        if (_verbose == true)
        {
            printf("Device handle closed.\n");
        }
    }

    if (_dev != NULL)
    {
        uvc_unref_device(_dev);
        if (_verbose == true)
        {
            printf("Device dereferenced.\n");
        }
    }

    if (_ctx != NULL)
    {
        uvc_exit(_ctx);
        if (_verbose == true)
        {
            printf("UVC Closed.\n");
        }
    }
}

bool UvcManager::GetAttribute(uint8_t unit_id, uint8_t control_id, void *attributePtr, uint16_t attributeWordLength)
{
    _uvc_lock.lock();
    int result = uvc_get_ctrl(_devh, unit_id, control_id, attributePtr, attributeWordLength, UVC_GET_CUR);
    _uvc_lock.unlock();

    if (result != attributeWordLength)
    {
        return false;
    }
    return true;
}

bool UvcManager::SetAttribute(uint8_t unit_id, uint8_t control_id, void *attributePtr, uint16_t attributeWordLength)
{
    _uvc_lock.lock();
    int result = uvc_set_ctrl(_devh, unit_id, control_id, attributePtr, attributeWordLength);
    _uvc_lock.unlock();

    if (result != attributeWordLength)
    {
        return false;
    }
    return true;
}

bool UvcManager::RunCommand(uint8_t unit_id, uint8_t control_id)
{
    leptonCommandIdToUnitId(unit_id);
    _uvc_lock.lock();
    int result = uvc_set_ctrl(_devh, unit_id, control_id, &control_id, 1);
    _uvc_lock.unlock();

    if (result != 1)
    {
        return false;
    }
    return true;
}


LEP_RESULT UVC_GetAttribute(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                            LEP_COMMAND_ID commandID,
                            LEP_ATTRIBUTE_T_PTR attributePtr,
                            LEP_UINT16 attributeWordLength)
{
    uint8_t unit_id = leptonCommandIdToUnitId(commandID);
    if (unit_id < 0)
    {
        return LEP_COMM_ERROR_READING_COMM;
    }

    uint8_t control_id = ((commandID & 0x00ff) >> 2) + 1;

    // Size in 16-bit words needs to be in bytes
    attributeWordLength *= 2;

    bool success = static_cast<UvcManager *>(portDescPtr->userPtr)->GetAttribute(unit_id, control_id, attributePtr,
                                                                                 attributeWordLength);
    if (success == true)
    {
        return LEP_OK;
    }
    return LEP_COMM_ERROR_READING_COMM;
}

LEP_RESULT UVC_SetAttribute(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                            LEP_COMMAND_ID commandID,
                            LEP_ATTRIBUTE_T_PTR attributePtr,
                            LEP_UINT16 attributeWordLength)
{
    uint8_t unit_id = leptonCommandIdToUnitId(commandID);
    if (unit_id < 0)
    {
        return LEP_COMM_ERROR_READING_COMM;
    }

    uint8_t control_id = ((commandID & 0x00ff) >> 2) + 1;

    // Size in 16-bit words needs to be in bytes
    attributeWordLength *= 2;

    bool success = static_cast<UvcManager *>(portDescPtr->userPtr)->SetAttribute(unit_id, control_id, attributePtr,
                                                                                 attributeWordLength);
    if (success == true)
    {
        return LEP_OK;
    }
    return LEP_COMM_ERROR_READING_COMM;
}

LEP_RESULT UVC_RunCommand(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                          LEP_COMMAND_ID commandID)
{
    uint8_t unit_id = leptonCommandIdToUnitId(commandID);
    if (unit_id < 0)
    {
        return LEP_COMM_ERROR_READING_COMM;
    }

    uint8_t control_id = ((commandID & 0x00ff) >> 2) + 1;

    bool success = static_cast<UvcManager *>(portDescPtr->userPtr)->RunCommand(unit_id, commandID);
    if (success == true)
    {
        return LEP_OK;
    }
    return LEP_COMM_ERROR_READING_COMM;
}

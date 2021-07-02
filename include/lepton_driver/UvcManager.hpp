
#ifndef LEPTON_DRIVER_UVCMANAGER_HPP
#define LEPTON_DRIVER_UVCMANAGER_HPP

#include <cstdint>
#include <mutex>

#include <libuvc/libuvc.h>

#include "lepton_sdk/LEPTON_SDK.h"

/**
 * @brief UvcManager manages a UVC device using C++ and Object Oriented design.
 */
class UvcManager
{
public:
    /**
     * @brief Constructor for UvcManager. Initializes everything for the UVC connection.
     * @param device_vid VID of the device being used.
     * @param device_pid PID of the device being used.
     * @param resolution_width Width of the image being captured.
     * @param resolution_height Height of the image being captured.
     * @param uvc_format Format of the image being captured.
     * @param callback Optional callback that will be called every time an image is captured. If this is nullptr, then no callback is used.
     * @param serial_number Optional serial number used to open a specific device. If this is nullptr, VID and PID are used to open the device.
     * @param verbose Sets the verbosity of the UvcManaged, useful for debugging.
     */
    UvcManager(uint16_t device_vid, uint16_t device_pid, uint16_t resolution_width, uint16_t resolution_height,
               uvc_frame_format uvc_format, std::function<void(uvc_frame_t*)> callback, const char* serial_number, bool verbose);

    /**
     * @brief Destructor for UvcManager. Severs the UVC connection before deallocation.
     */
    ~UvcManager();

    /**
     * @brief Stop streaming data from the camera.
     */
    void StopStream();

    /**
     * @brief Start streaming data from the camera.
     * @return True if the camera started streaming, false if it failed.
     */
    bool StartStream();

    /**
     * @brief Sets the video format to the provided values
     * @param resolution_width Width of the image being captured.
     * @param resolution_height Height of the image being captured.
     * @param uvc_format Format of the image being captured.
     * @return True if the new video format was applied successfully, false otherwise.
     */
    bool SetVideoFormat(uint16_t resolution_width, uint16_t resolution_height, uvc_frame_format uvc_format);

    /**
     * @brief Gets the most recently captured image from the camera as a OpenCV Mat.
     * @return OpenCV Mat of the most recent frame.
     */
    cv::Mat GetLastFrame();

    /**
     * @brief Gets an attribute from the UVC Device.
     * @param unit_id The unit ID of the attribute being read.
     * @param control_id The control ID of the attribute being read.
     * @param attribute_ptr Pointer to where the attribute value will be copied into.
     * @param attribute_size Size of the object attribute_ptr points to.
     * @return True if the attribute was successfully read, false otherwise.
     */
    bool GetAttribute(uint8_t unit_id, uint8_t control_id, void* attribute_ptr, uint16_t attribute_size);

    /**
     * @brief Sets an attribute on the UVC Device.
     * @param unit_id The unit ID of the attribute being modified.
     * @param control_id The control ID of the attribute being modified.
     * @param attribute_ptr Pointer to the value that the attribute will be set to.
     * @param attribute_length Size of the object attribute_ptr points to.
     * @return True if the attribute was successfully updated, false otherwise.
     */
    bool SetAttribute(uint8_t unit_id, uint8_t control_id, void* attribute_ptr, uint16_t attribute_size);

    /**
     * @brief Runs a command on the UVC Device.
     * @param unit_id The unit ID of the command being run.
     * @param control_id The control ID of the command being run.
     * @return True if the command was successfully run, false otherwise.
     */
    bool RunCommand(uint8_t unit_id, uint8_t control_id);

private:
    /**
     * @brief Pointer to the context of the UVC device.
     */
    uvc_context_t *_ctx;

    /**
     * @brief Pointer to the UVC device.
     */
    uvc_device_t *_dev;

    /**
     * @brief Pointer to the UVC device handle.
     */
    uvc_device_handle_t *_devh;

    /**
     * @brief UVC device stream controller.
     */
    uvc_stream_ctrl_t _ctrl;
    
    /**
     * @brief Serial Number of the device. Optionally used to open a device with a specific serial number. NULL if not used.
     */
    const char* _serial_number;

    /**
     * @brief Mutex for reading and writing _last_frame.
     */
    std::mutex _last_frame_lock;

    /**
     * @brief Mutex for sending and receiving data from the UVC device.
     */
    std::mutex _uvc_lock;

    /**
     * @brief OpenCV Mat of the last captured image.
     * @note This should be read with GetLastFrame() and SetLastFrame(), which handle the mutex.
     */
    cv::Mat _last_frame;

    /**
     * @brief VID of the device to capture from.
     */
    uint16_t _device_vid;

    /**
     * @brief PID of the device to capture from.
     */
    uint16_t _device_pid;

    /**
     * @brief Width of the image being captured in pixels.
     */
    uint16_t _resolution_width;

    /**
     * @brief Height of the image being captured in pixels.
     */
    uint16_t _resolution_height;

    /**
     * @brief Format of image being captured by UVC.
     */
    uvc_frame_format _uvc_format;

    /**
     * @brief Flag to print extra information.
     */
    bool _verbose;

    /**
     * @brief Initializes UVC.
     * @return True if UVC was successfully initialized, false otherwise.
     */
    bool InitUvc();

    /**
     * @brief Finds the UVC device with _device_vid and _device_pid.
     * @return True if the device was found, false otherwise.
     */
    bool FindDevice();

    /**
     * @brief Opens the UVC device with _device_vid and _device_pid.
     * @return True if the device was opened, false otherwise.
     */
    bool OpenDevice();

    /**
     * @brief Formats the device with _uvc_format, _resolution_width, and _resolution_height.
     * @return True if the device was formatted, false otherwise.
     */
    bool FormatDevice();

    /**
     * @brief Severs the UVC connection. This is separate from the destructor because it may need to be called in the
     * constructor if construction fails partway though, for example opening the device is successful but formatting is
     * not.
     */
    void Cleanup();

    /**
     * @brief Custom callback function that is called whenever UVC captures an image. If this is nullptr, then no custom
     * callback is called.
     */
    std::function<void(uvc_frame_t*)> CaptureCallback;

    /**
     * @brief Sets the _last_frame to the input, and handles mutexes.
     * @param input_frame Frame that _last_frame will be set to.
     */
    void SetLastFrame(cv::Mat input_frame);

    /**
     * @brief Friend function that UVC calls when it captures an image.
     * @param frame Pointer to the UVC frame that was captured.
     * @param ptr Pointer to the UvcManager that manages the UVC device that captured the frame.
     * @note This must be a friend function to access some private variables.
     */
    friend void uvc_callback(uvc_frame_t *frame, void *ptr);
};

#endif //LEPTON_DRIVER_UVCMANAGER_HPP

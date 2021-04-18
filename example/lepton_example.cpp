
#include <iostream>
#include <thread>
#include <chrono>

#include "opencv2/core.hpp"
#include "cv.hpp"

#include "lepton_driver/LeptonCamera.hpp"
#include "lepton_sdk/LEPTON_ErrorCodes.h"

int main()
{
    // Create the camera. Everything for the camera to start is done here, and it will automatically start capturing
    // frames after creation. The port_id can be anything, so long as nothing shares the port_id. capture_bits can be
    // LEPTON_CAPTURE_8_BIT or LEPTON_CAPTURE_14_BIT. 14 bit radiometric data contains more information, but has very
    // low contrast, so for typical viewing, use LEPTON_CAPTURE_8_BIT. Callback is nullprt since we grab the images
    // using LeptonCamera::GetLastFrame(), and verbose is true so we can see more information.
    LeptonCamera cam(5, LEPTON_CAPTURE_8_BIT, nullptr, true);

    // Wait two seconds, this allows the camera to initialize. Rarely UVC devices can take longer than this to
    // initialize
    std::this_thread::sleep_for(std::chrono::seconds(2));
    int key = '0';
    while (key != 'x')
    {
        // Press x to exit

        // Get the most recent image
        cv::Mat image = cam.GetLastFrame();
        if (image.size == 0)
        {
            // Image is empty, which can happen if we try to grab the image before uvc gets it from the camera.
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        cv::imshow("main", cam.GetLastFrame());
        key = cv::waitKey(10);
        if (key == '1')
        {
            // Pressing 1 will set the Lepton to 14 bit
            bool success = cam.SetCaptureBits(LEPTON_CAPTURE_14_BIT);
            if (success == false)
            {
                std::cout << "Set to 14 bits Fail\n";
                break;
            }
        } else if (key == '8')
        {
            // Pressing 8 will set the lepton to 8 bit
            bool success = cam.SetCaptureBits(LEPTON_CAPTURE_8_BIT);
            if (success == false)
            {
                std::cout << "Set to 8 bits Fail\n";
                break;
            }
        } else if (key == 'a')
        {
            // Pressing a will turn AGC On
            LEP_RESULT result = cam.SetAgcEnableState(LEP_AGC_ENABLE);
            if (result != LEP_OK)
            {
                std::cout << "AGC enable Fail\n";
                break;
            }
        } else if (key == 's')
        {
            // Pressing s will turn AGC Off. This makes 8 bit images look bad because they convert from 14 to 8 bits by
            // dropping the most significant byte.
            LEP_RESULT result = cam.SetAgcEnableState(LEP_AGC_DISABLE);
            if (result != LEP_OK)
            {
                std::cout << "AGC disable Fail\n";
                break;
            }
        } else if (key == 'f')
        {
            // Pressing f perform FFC.
            LEP_RESULT result;
            result = cam.RunSysFFCNormalization();
            if (result != LEP_OK)
            {
                std::cout << "FFC Fail\n";
                break;
            }
        }
    }
    return 0;
}
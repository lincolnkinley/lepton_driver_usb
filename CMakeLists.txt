cmake_minimum_required(VERSION 3.0.2)
project(lepton_driver)

set (CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(lepton_driver
        src/LeptonCamera.cpp
        src/UvcManager.cpp)

target_include_directories(lepton_driver PUBLIC include)
target_link_libraries(lepton_driver PUBLIC ${OpenCV_LIBS})

add_library(LeptonSDK
        lepton_sdk/lepton_sdk/crc16fast.c
        lepton_sdk/lepton_sdk/LEPTON_AGC.c
        lepton_sdk/lepton_sdk/LEPTON_OEM.c
        lepton_sdk/lepton_sdk/LEPTON_RAD.c
        lepton_sdk/lepton_sdk/LEPTON_SDK.c
        lepton_sdk/lepton_sdk/LEPTON_SYS.c
        lepton_sdk/lepton_sdk/LEPTON_VID.c
        lepton_sdk/lepton_sdk/uvc_sdk.c
        )
target_include_directories(LeptonSDK PUBLIC lepton_sdk)
target_link_libraries(LeptonSDK PUBLIC uvc)

target_link_libraries(lepton_driver PUBLIC LeptonSDK)

add_executable(LeptonExample example/lepton_example.cpp)
target_link_libraries(LeptonExample lepton_driver LeptonSDK)


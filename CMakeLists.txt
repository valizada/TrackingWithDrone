cmake_minimum_required(VERSION 3.3)
project(sample_tracking)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(OpenCV_INCLUDE_DIRS /usr/local/opt/opencv3/include)
set(CMAKE_EXE_LINKER_FLAGS -L/usr/local/opt/opencv3/lib)

set(SOURCE_FILES
        samples/sample_tracking.cpp
        src/ardrone/ardrone.cpp
        src/ardrone/ardrone.h
        src/ardrone/command.cpp
        src/ardrone/config.cpp
        src/ardrone/navdata.cpp
        src/ardrone/tcp.cpp
        src/ardrone/udp.cpp
        src/ardrone/uvlc.h
        src/ardrone/version.cpp
        src/ardrone/video.cpp)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(sample_tracking ${SOURCE_FILES})

target_link_libraries(sample_tracking -lm  -lpthread -lavutil -lavformat
        -lavcodec
        -lswscale
        -lopencv_calib3d
        -lopencv_core
        -lopencv_features2d
        -lopencv_flann
        -lopencv_highgui
        -lopencv_imgcodecs
        -lopencv_imgproc
        -lopencv_ml
        -lopencv_objdetect
        -lopencv_photo
        -lopencv_stitching
        -lopencv_superres
        -lopencv_video
        -lopencv_videoio
        -lopencv_videostab)


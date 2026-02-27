cmake_minimum_required(VERSION 3.4.1)

project(rknn_yolov5_demo)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-pthread")

# skip 3rd-party lib dependencies
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--allow-shlib-undefined")

# install target and libraries
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install/rknn_yolov5_demo_${CMAKE_SYSTEM_NAME})

set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# rknn api
set(RKNN_API_PATH ${CMAKE_SOURCE_DIR}/../../runtime/RK3588/${CMAKE_SYSTEM_NAME}/librknn_api)
set(LIB_ARCH aarch64)
set(RKNN_RT_LIB ${CMAKE_SOURCE_DIR}/include/librknnrt.so)

#rga
set(RGA_PATH ${CMAKE_SOURCE_DIR}/include/3rdparty/rga/RK3588)
set(RGA_LIB ${RGA_PATH}/lib/Linux//${LIB_ARCH}/librga.so)


include_directories(${RKNN_API_PATH}/include)
include_directories(${CMAKE_SOURCE_DIR}/include/3rdparty)
include_directories(${RGA_PATH}/include)


find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0>=1.14)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GST_RTSP REQUIRED gstreamer-rtsp-server-1.0)
# opencv
find_package(OpenCV REQUIRED)

include_directories(${GST_INCLUDE_DIRS} ${GST_APP_INCLUDE_DIRS} ${GST_RTSP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${GST_LIBRARY_DIRS} ${GST_APP_LIBRARY_DIRS} ${GST_RTSP_LIBRARY_DIRS})


add_definitions(${GST_CFLAGS_OTHER})
add_definitions(${GST_APP_CFLAGS_OTHER})
add_definitions(${GST_RTSP_CFLAGS_OTHER})

set(CMAKE_INSTALL_RPATH "lib")


# rknn_yolov5_demo
include_directories( ${CMAKE_SOURCE_DIR}/include)

add_executable(rknn_yolov5_demo
        src/main.cc
        src/common.cc
        src/worker_thread.cc
        src/rtsp_streamer.cc
        detect/postprocess.cc
        detect/preprocess.cc
        detect/rkYolov5s.cc
        bytetrack/BYTETracker.cpp
        bytetrack/kalmanFilter.cpp
        bytetrack/lapjv.cpp
        bytetrack/STrack.cpp
        bytetrack/utils.cpp
        bytetrack/rknn_utils.cc
        servo/servoControl.cc
        servo/station_guidance.cc
        servo/stationManager.cc
)

target_link_libraries(rknn_yolov5_demo
  ${RKNN_RT_LIB}
  ${OpenCV_LIBS}
  ${RGA_LIB}
  ${GST_LIBRARIES}
  ${GST_APP_LIBRARIES}
  ${GST_RTSP_LIBRARIES}
)


# install target and libraries
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install/rknn_yolov5_demo_${CMAKE_SYSTEM_NAME})
install(TARGETS rknn_yolov5_demo DESTINATION ./)
install(PROGRAMS ${RKNN_RT_LIB} DESTINATION lib)
install(PROGRAMS ${RGA_LIB} DESTINATION lib)
install(DIRECTORY model DESTINATION ./)

message(STATUS "GStreamer include dirs: ${GST_INCLUDE_DIRS}")
message(STATUS "GStreamer app include dirs: ${GST_APP_INCLUDE_DIRS}")
message(STATUS "GStreamer RTSP include dirs: ${GST_RTSP_INCLUDE_DIRS}")
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "RKNN runtime lib used: ${RKNN_RT_LIB}")
message(STATUS "RGA lib used: ${RGA_LIB}")
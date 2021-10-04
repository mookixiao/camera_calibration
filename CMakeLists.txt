cmake_minimum_required(VERSION 3.17)
project(camera_calibration)

set(CMAKE_CXX_STANDARD 11)

set(OpenCV_DIR "D:/opencv-4.5.3/build/install/x64/mingw/lib")
find_package(OpenCV REQUIRED)

add_executable(camera_calibration main.cpp)

target_link_libraries(
        camera_calibration
        PRIVATE ${OpenCV_LIBS}
)

target_include_directories(
        camera_calibration
        PRIVATE ${OpenCV_INCLUDE_DIRS}
)
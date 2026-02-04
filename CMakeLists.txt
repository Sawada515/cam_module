cmake_minimum_required(VERSION 3.20)

project(cam_module LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)")
    add_compile_options(-O3)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^arm")
    add_compile_options(-mfpu=neon -mfloat-abi=hard -O3)
endif()

#python3
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

#pybind11
find_package(pybind11 REQUIRED)

#opencv
find_package(OpenCV REQUIRED)

#turbojpeg
find_package(PkgConfig REQUIRED)
pkg_check_modules(TURBOJPEG REQUIRED libturbojpeg)

#spdlog
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

#Threads
find_package(Threads REQUIRED)

#yaml-cpp
find_package(yaml-cpp REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

set(SOURCE_FILES
    src/main.cpp
    src/camera/v4l2_capture.cpp
    src/camera/capture_thread.cpp
    src/image_processor/image_processor.cpp
    src/json/json_client.cpp
    src/json/json_codec.cpp
    src/network/resolve_hostname.cpp
    src/network/udp/udp_socket.cpp
    src/network/tcp/tcp_socket.cpp
    src/network/tcp/tcp_thread.cpp
    src/read_config/read_config.cpp
    src/read_resistor_value/read_resistor_value.cpp
    src/send_image/send_image.cpp
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${TURBOJPEG_INCLUDE_DIRS}
    ${pybind11_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    ${OpenCV_LIBS}
    ${TURBOJPEG_LIBRARIES}
    pybind11::embed
    Python3::Python
    spdlog::spdlog
    fmt::fmt
    Threads::Threads
)

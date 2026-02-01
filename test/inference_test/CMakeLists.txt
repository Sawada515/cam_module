cmake_minimum_required(VERSION 3.20)
project(image_processor_test CXX)

# --- コンパイラ設定 (C++20) ---
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

# --- 仮想環境 (venv) の優先設定 ---
set(Python3_FIND_VIRTUALENV FIRST)

# --- パッケージの検索 ---
# 1. Python3
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# 2. Pybind11
find_package(pybind11 REQUIRED)

# 3. OpenCV
find_package(OpenCV REQUIRED)

# 4. TurboJPEG
find_package(PkgConfig REQUIRED)
pkg_check_modules(TURBOJPEG REQUIRED libturbojpeg)

# 5. Spdlog & fmt (追加: ログ出力用ライブラリのリンクが必要)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

# --- インクルードディレクトリの設定 ---
include_directories(
    ${CMAKE_SOURCE_DIR}/../../include
    ${OpenCV_INCLUDE_DIRS}
    ${TURBOJPEG_INCLUDE_DIRS}
    ${pybind11_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
)

# --- ソースファイルの指定 ---
set(SOURCES
    # メインのテストファイル (ファイル名が inference_test.cpp になっている場合はここを書き換えてください)
    inference_test.cpp 

    # 依存する実装ファイル
    ../../src/camera/capture_thread.cpp
    ../../src/camera/v4l2_capture.cpp
    ../../src/image_processor/image_processor.cpp
    ../../src/json/json_codec.cpp
    
    # 追加: ReadResistorValueの実装ファイル
    ../../src/read_resistor_value/read_resistor_value.cpp
)

# --- 実行ファイルの作成 ---
add_executable(test_image_processor ${SOURCES})

# --- ライブラリのリンク ---
target_link_libraries(test_image_processor
    PRIVATE
    ${OpenCV_LIBS}
    ${TURBOJPEG_LIBRARIES}
    pybind11::embed
    Python3::Python
    spdlog::spdlog   # 追加
    fmt::fmt         # 追加
    pthread
)

# --- 確認用メッセージ ---
message(STATUS "Python3 Executable: ${Python3_EXECUTABLE}")
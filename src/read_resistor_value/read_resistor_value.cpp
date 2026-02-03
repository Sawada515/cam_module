/**
 * @file read_resistor_value.cpp
 * @brief 抵抗値の読み取り
 * @author sawada
 * @date 2026-01-30
 */

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <python3.13/pylifecycle.h>

#include <cstdlib>
#include <vector>
#include <stdexcept>
#include <filesystem>
#include <mutex>
#include <atomic>

#include <unistd.h>
#include <limits.h>

#include "read_resistor_value/read_resistor_value.hpp"
#include "logger/logger.hpp"

namespace py = pybind11;
namespace fs = std::filesystem;

namespace {
    class PythonRuntime {
        public:
            PythonRuntime(const PythonRuntime&) = delete;
            PythonRuntime& operator=(const PythonRuntime&) = delete;

            explicit PythonRuntime(const std::string& venv_path, const std::string& script_dir) {
                try {
                    fs::path venv_root_path(fs::absolute(venv_path));
                    fs::path script_path(fs::absolute(script_dir));
    
                    //python_home_ = decode_str(venv_root_path.string());
    
                    //Py_SetPythonHome(python_home_.get());
    
                    fs::path python_exe_path = venv_root_path / "bin" / "python";
    
                    program_name_ = decode_str(python_exe_path.string());
    
                    Py_SetProgramName(program_name_.get());
    
                    guard_ = std::make_unique<py::scoped_interpreter>();
    
                    {
                        py::gil_scoped_acquire acquire;
    
                        py::module sys = py::module::import("sys");

                        sys.attr("path").attr("append")(script_path.string());
                    }
                }
                catch (const std::exception& e) {
                    spdlog::error("Failed to python runtime init error {}", e.what());

                    throw;
                }
            }

            ~PythonRuntime() = default;

        private:
            struct PyMemDeleter {
                void operator()(wchar_t* p) const {
                    if (p) {
                        PyMem_RawFree(p);
                    }
                }
           };

            std::unique_ptr<wchar_t[], PyMemDeleter> program_name_;
            std::unique_ptr<wchar_t[], PyMemDeleter> python_home_;

            std::unique_ptr<py::scoped_interpreter> guard_;

            std::unique_ptr<wchar_t[], PyMemDeleter> decode_str(const std::string& s)
            {
                wchar_t* decoded = Py_DecodeLocale(s.c_str(), nullptr);
                if (!decoded) {
                    throw std::runtime_error("Failed to docode path string: " + s);
                }

                return std::unique_ptr<wchar_t[], PyMemDeleter>(decoded);
            }
    };

    static std::atomic<PythonRuntime*> g_runtime_instance{nullptr};

    static std::mutex g_init_mutex;

    static std::string g_initialized_venv_path;
}

void initialize_python_runtime(const std::string& venv_path, const std::string& script_path)
{
    std::lock_guard<std::mutex> lock(g_init_mutex);

    if (g_runtime_instance.load() != nullptr) {
        if (g_initialized_venv_path != venv_path) {
            throw std::logic_error("PythonRuntime error No re-init initialize_python_runtime()");
        }
        else {
            throw std::logic_error("PythonRuntime error already initialized");
        }
    }

    static PythonRuntime runtime(venv_path, script_path);

    g_initialized_venv_path = venv_path;
    g_runtime_instance.store(&runtime);
}

static PythonRuntime& get_runtime_instance()
{
    PythonRuntime* ptr = g_runtime_instance.load();

    if (!ptr) {
        throw std::logic_error("Error PythonRuntime no initialized");
    }

    return *ptr;
}

static py::array_t<uint8_t> mat_to_numpy(const cv::Mat& src)
{
    if (src.empty()) {
        return py::array_t<uint8_t>();
    }

    std::vector<ssize_t> shape = { src.rows, src.cols, src.channels() };
    std::vector<ssize_t> strides {
        static_cast<ssize_t>(src.step[0]),
        static_cast<ssize_t>(src.step[1]),
        static_cast<ssize_t>(src.elemSize1())
    };

    return py::array_t<uint8_t>(shape, strides, src.data).attr("copy")();
}

struct __attribute__((visibility("hidden"))) ReadResistorValue::Impl {
    py::object py_instance;

    Impl(const std::string& band_path, const std::string& color_path) {
        get_runtime_instance();

        py::gil_scoped_acquire acquire;

        try {
            py::module module = py::module::import("read_resistor_value");

            py_instance = module.attr("InferenceResistorValue")(band_path, color_path);
        }
        catch (const py::error_already_set& e) {
            std::string error_msg = "[ReadResistorValue] Python Class Init Failed:\n" + std::string(e.what());

            spdlog::error("{}", error_msg);

            throw std::runtime_error(error_msg);
        }
    }

    std::optional<double> inference(const cv::Mat& roi) {
        py::gil_scoped_acquire acquire;
        try {
            auto np_img = mat_to_numpy(roi);

            py::object result = py_instance.attr("inference")(np_img);

            if (result.is_none()) {
                return std::nullopt;
            }

            return result.cast<double>();
        }
        catch (const py::error_already_set& e) {
            spdlog::error("Inference error : {}", e.what());

            return std::nullopt;
        }
    }
};

ReadResistorValue::ReadResistorValue(const std::string& band_path, const std::string& color_path)
    : pImpl(std::make_unique<Impl>(band_path, color_path)) 
{
}

ReadResistorValue::~ReadResistorValue() = default;

ReadResistorValue::ReadResistorValue(ReadResistorValue&&) noexcept = default;
ReadResistorValue& ReadResistorValue::operator=(ReadResistorValue&&) noexcept = default;

std::optional<double> ReadResistorValue::inference(const cv::Mat& resistor_roi) {
    if (!pImpl) {
        return std::nullopt;
    }

    return pImpl->inference(resistor_roi);
}

#ifdef APA_PLATFORM_X86

#include "PythonPlot_Api.hpp"
#include <Python.h>
#include <map>
#include <mutex>
#include <iostream>
#include <memory>
#include <cmath>

namespace
{
std::map<std::string, std::unique_ptr<PythonPlot>> g_instances;
std::mutex g_mutex;
bool g_python_initialized = false;
} // namespace

PythonPlot &PythonPlot::Instance(const std::string &module)
{
    std::lock_guard<std::mutex> lock(g_mutex);

    if (!g_python_initialized)
    {
        Py_Initialize();
        if (!Py_IsInitialized())
        {
            std::cerr << "Failed to initialize Python interpreter!" << std::endl;
            std::abort();
        }
        g_python_initialized = true;
    }

    auto it = g_instances.find(module);
    if (it == g_instances.end())
    {
        g_instances[module] = std::unique_ptr<PythonPlot>(new PythonPlot(module));
        it                  = g_instances.find(module);
    }

    return *(it->second);
}

PythonPlot::PythonPlot(const std::string &module)
    : module_(module), min_x_(0), max_x_(0), min_y_(0), max_y_(0), has_data_(false)
{
    PyRun_SimpleString("import matplotlib.pyplot as plt");
    PyRun_SimpleString("import numpy as np");
    std::string cmd = "plt.figure('" + module + "')\nplt.axis('equal')";
    PyRun_SimpleString(cmd.c_str());
}

PythonPlot::~PythonPlot() {}

void PythonPlot::Clear()
{
    PyRun_SimpleString("import matplotlib.pyplot as plt\nplt.clf()");
    // 不重置范围，以实现“只外扩不回缩”的行为
}

void PythonPlot::UpdateBounds(double x, double y)
{
    if (!has_data_)
    {
        min_x_ = max_x_ = x;
        min_y_ = max_y_ = y;
        has_data_       = true;
    }
    else
    {
        if (x < min_x_)
            min_x_ = x;
        if (x > max_x_)
            max_x_ = x;
        if (y < min_y_)
            min_y_ = y;
        if (y > max_y_)
            max_y_ = y;
    }
}

void PythonPlot::PlotLine(const std::vector<double> &x, const std::vector<double> &y,
                          const std::string &style)
{
    if (x.empty() || y.empty() || x.size() != y.size())
        return;

    for (size_t i = 0; i < x.size(); ++i) UpdateBounds(x[i], y[i]);

    PyObject *py_x = PyList_New(x.size());
    PyObject *py_y = PyList_New(y.size());
    for (size_t i = 0; i < x.size(); ++i)
    {
        PyList_SetItem(py_x, i, PyFloat_FromDouble(x[i]));
        PyList_SetItem(py_y, i, PyFloat_FromDouble(y[i]));
    }

    PyObject *main_module = PyImport_AddModule("__main__");
    PyObject *main_dict   = PyModule_GetDict(main_module);
    PyDict_SetItemString(main_dict, "x", py_x);
    PyDict_SetItemString(main_dict, "y", py_y);

    std::string cmd = "import matplotlib.pyplot as plt\nplt.plot(x, y, '" + style + "')";
    PyRun_SimpleString(cmd.c_str());

    Py_DECREF(py_x);
    Py_DECREF(py_y);
}

void PythonPlot::PlotPolygon(const std::vector<std::pair<double, double>> &points,
                             const std::string &color, double alpha)
{
    if (points.empty())
        return;

    PyObject *py_x = PyList_New(points.size() + 1);
    PyObject *py_y = PyList_New(points.size() + 1);

    for (size_t i = 0; i < points.size(); ++i)
    {
        PyList_SetItem(py_x, i, PyFloat_FromDouble(points[i].first));
        PyList_SetItem(py_y, i, PyFloat_FromDouble(points[i].second));
        UpdateBounds(points[i].first, points[i].second);
    }

    // 追加首点闭合
    PyList_SetItem(py_x, points.size(), PyFloat_FromDouble(points[0].first));
    PyList_SetItem(py_y, points.size(), PyFloat_FromDouble(points[0].second));

    PyObject *main_module = PyImport_AddModule("__main__");
    PyObject *main_dict   = PyModule_GetDict(main_module);
    PyDict_SetItemString(main_dict, "px", py_x);
    PyDict_SetItemString(main_dict, "py", py_y);

    char buf[256];
    snprintf(buf, sizeof(buf),
             "import matplotlib.pyplot as plt\n"
             "plt.plot(px, py, color='%s', alpha=%f, linewidth=2.0)",
             color.c_str(), alpha);
    PyRun_SimpleString(buf);

    Py_DECREF(py_x);
    Py_DECREF(py_y);
}

void PythonPlot::PlotText(double x, double y, const std::string &text)
{
    UpdateBounds(x, y);
    char buf[256];
    snprintf(buf, sizeof(buf),
             "import matplotlib.pyplot as plt\n"
             "plt.text(%f, %f, '%s')",
             x, y, text.c_str());
    PyRun_SimpleString(buf);
}

void PythonPlot::PlotCar(double x, double y, double yaw, double length, double width,
                         double rear_to_center, const std::string &color, double alpha)
{
    double front_to_center = length - rear_to_center;
    double half_width      = width / 2.0;

    std::vector<std::pair<double, double>> corners_local = {
        {-rear_to_center, half_width},
        {-rear_to_center, -half_width},
        {front_to_center, -half_width},
        {front_to_center, half_width}};

    std::vector<std::pair<double, double>> corners_global;
    corners_global.reserve(4);
    for (auto &p : corners_local)
    {
        double gx = x + p.first * std::cos(yaw) - p.second * std::sin(yaw);
        double gy = y + p.first * std::sin(yaw) + p.second * std::cos(yaw);
        corners_global.emplace_back(gx, gy);
    }

    PlotPolygon(corners_global, color, alpha);

    // --- 在后轴中心标注坐标（保留两位小数） ---
    char text_buf[64];
    snprintf(text_buf, sizeof(text_buf), "(%.2f, %.2f)", x, y);
    PlotText(x, y, text_buf);
}

void PythonPlot::Show()
{
    if (!has_data_)
    {
        PyRun_SimpleString("import matplotlib.pyplot as plt\nplt.pause(0.001)");
        return;
    }

    char buf[512];
    snprintf(buf, sizeof(buf),
             "import matplotlib.pyplot as plt\n"
             "plt.xlim(%f, %f)\n"
             "plt.ylim(%f, %f)\n"
             "plt.gca().set_aspect('equal', adjustable='box')\n"
             "plt.grid(True)\n"
             "plt.pause(0.001)",
             min_x_ - 0.5, max_x_ + 0.5, min_y_ - 0.5, max_y_ + 0.5);
    PyRun_SimpleString(buf);
}

#endif // APA_PLATFORM_X86

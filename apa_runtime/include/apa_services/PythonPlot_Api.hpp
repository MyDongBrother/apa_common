#pragma once
#include <vector>
#include <string>
#include <utility>
#include <cstdint>

#ifdef APA_PLATFORM_X86

/**
 * @brief Python 可视化接口封装类（仅在 X86 平台启用）
 */
class PythonPlot
{
  public:
    static PythonPlot &Instance(const std::string &module);

    void Clear();
    void PlotLine(const std::vector<double> &x, const std::vector<double> &y,
                  const std::string &style = "b-");
    void PlotPolygon(const std::vector<std::pair<double, double>> &points,
                     const std::string &color = "g", double alpha = 0.3);
    void PlotText(double x, double y, const std::string &text);

    void PlotCar(double x, double y, double yaw, double length, double width,
                 double rear_to_center, const std::string &color = "r",
                 double alpha = 1.0);

    void Show();
    ~PythonPlot();

  private:
    explicit PythonPlot(const std::string &module);

    PythonPlot(const PythonPlot &)            = delete;
    PythonPlot &operator=(const PythonPlot &) = delete;

    std::string module_;

    void UpdateBounds(double x, double y);

    double min_x_, max_x_, min_y_, max_y_;
    bool has_data_;
};

#else
class PythonPlot
{
  public:
    static PythonPlot &Instance(const std::string &)
    {
        static PythonPlot dummy;
        return dummy;
    }
    void Clear() {}
    void PlotLine(const std::vector<double> &, const std::vector<double> &,
                  const std::string & = "b-")
    {
    }
    void PlotPolygon(const std::vector<std::pair<double, double>> &,
                     const std::string & = "g", double = 1.0)
    {
    }
    void PlotText(double, double, const std::string &) {}
    void PlotCar(double, double, double, double, double, double,
                 const std::string & = "r", double = 1.0)
    {
    }
    void Show() {}
    ~PythonPlot() {}

  private:
    PythonPlot() = default;
};

#endif // APA_PLATFORM_X86

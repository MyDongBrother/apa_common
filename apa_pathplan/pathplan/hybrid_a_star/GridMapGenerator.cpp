#include "GridMapGenerator.h"
#include "hybrid_data.h"
#ifndef ndk
#include <opencv2/opencv.hpp> // 包含 OpenCV 头文件
#endif
#include "json/json.h"
using namespace std;

// Lambda 函数用于校正接近零的值
auto correct_to_zero = [](double value) -> double {
    return std::abs(value) < 1e-4 ? 0.0 : value;
};

// 构造函数初始化
GridMapGenerator::GridMapGenerator(double resolution, double x_min, double x_max,
                                   double y_min, double y_max,
                                   const std::string &inputJson,
                                   const std::string &namePrex)
    : resolution(resolution),
      x_min(x_min),
      x_max(x_max),
      y_min(y_min),
      y_max(y_max),
      inputJsonData(inputJson),
      outputFilePath(namePrex)
{
    // 计算栅格地图的尺寸
    x_size = static_cast<int>(ceil((x_max - x_min) / resolution)) + 1;
    y_size = static_cast<int>(ceil((y_max - y_min) / resolution)) + 1;

    // 初始化栅格地图，0 表示可通行，1 表示障碍物
    gridMap = std::vector<std::vector<int>>(y_size, std::vector<int>(x_size, 0));
}

// 将物理坐标转换为栅格索引（整数）
std::pair<int, int> GridMapGenerator::toGridIdxInt(double x, double y) const
{
    int x_idx = round((x - x_min) / resolution);
    int y_idx = round((y - y_min) / resolution);
    return {x_idx, y_idx};
}

// 将物理坐标转换为栅格索引（浮点）
std::pair<double, double> GridMapGenerator::toGridIdxDouble(double x, double y) const
{
    double x_idx = (x - x_min) / resolution;
    double y_idx = (y - y_min) / resolution;
    return {x_idx, y_idx};
}

static void Convert(const double *pO, const double *pin, double *pout)
{
    pout[0] = pin[0] * cos(pO[2]) - pin[1] * sin(pO[2]) + pO[0];
    pout[1] = pin[0] * sin(pO[2]) + pin[1] * cos(pO[2]) + pO[1];
}

bool GridMapGenerator::parseJSON(std::vector<LineSeg_T> &obstaclelines)
{
    // 初始化动态范围
    double dynamic_x_min = std::numeric_limits<double>::max();
    double dynamic_x_max = std::numeric_limits<double>::lowest();
    double dynamic_y_min = std::numeric_limits<double>::max();
    double dynamic_y_max = std::numeric_limits<double>::lowest();

    // std::cout << "解析前的 JSON 数据: " << jsonStr << std::endl;
    Json::Reader jsonReader;
    Json::Value jsonData;
    jsonData.clear();
    if (!jsonReader.parse(inputJsonData, jsonData))
    {
        std::cerr << "解析 JSON 数据失败。" << inputJsonData << std::endl;
        return false;
    }

    // 检查是否存在 "objs" 字段并解析
    // 从 "objs" 字段中解析障碍物数据（每个子数组包含 x1, y1, x2, y2）
    if (jsonData.isMember("objs") && jsonData["objs"].isArray())
    {
        for (int i = 0; i < jsonData["objs"].size(); i++)
        {
            int j     = 0;
            double x1 = jsonData["objs"][i][j++].asDouble();
            double y1 = jsonData["objs"][i][j++].asDouble();
            double x2 = jsonData["objs"][i][j++].asDouble();
            double y2 = jsonData["objs"][i][j++].asDouble();

            dynamic_x_min = std::min(dynamic_x_min, x1);
            dynamic_x_max = std::max(dynamic_x_max, x1);
            dynamic_y_min = std::min(dynamic_y_min, y1);
            dynamic_y_max = std::max(dynamic_y_max, y1);
            dynamic_x_min = std::min(dynamic_x_min, x2);
            dynamic_x_max = std::max(dynamic_x_max, x2);
            dynamic_y_min = std::min(dynamic_y_min, y2);
            dynamic_y_max = std::max(dynamic_y_max, y2);

            // 每个障碍物作为一条独立的线段，保存为两个连续的点
            LineSeg_T objLine;
            objLine.pt1.x = x1;
            objLine.pt1.y = y1;
            objLine.pt2.x = x2;
            objLine.pt2.y = y2;

            obstaclelines.push_back(objLine);
        }
    }
    else
    {
        std::cerr << "JSON 数据中不包含 'objs' 字段。" << std::endl;
        return false;
    }

    // 检查是否存在 "slotobjs" 字段
    // 转换起点和终点
    if (jsonData.isMember("curpos") && jsonData["curpos"].isArray() &&
        jsonData["curpos"].size() >= 3)
    {
        for (int i = 0; i < (int)jsonData["curpos"].size(); ++i)
        {
            start.push_back(jsonData["curpos"][i].asDouble());
        }
    }
    else
    {
        std::cerr << "JSON 数据中不包含 'curpos' 字段。" << std::endl;
        return false;
    }

    if (jsonData.isMember("slottarg") && jsonData["slottarg"].isArray() &&
        jsonData["slottarg"].size() >= 3)
    {
        for (int i = 0; i < (int)jsonData["slottarg"].size(); ++i)
        {
            end.push_back(jsonData["slottarg"][i].asDouble());
        }
    }
    else
    {
        std::cerr << "JSON 数据中不包含 'slottarg' 字段。" << std::endl;
        return false;
    }

    if (jsonData.isMember("slotId"))
    {
        gridId = jsonData["slotId"].asInt();
    }
    else
    {
        std::cerr << "JSON 数据中不包含 'slotId' 字段。" << std::endl;
        return false;
    }

    // 根据起点、终点和障碍物点更新地图范围
    dynamic_x_min = std::max(dynamic_x_min, end[0] - 10.0f);
    dynamic_x_max = std::min(dynamic_x_max, end[0] + 10.0f);
    dynamic_y_min = std::max(dynamic_y_min, end[1] - 10.0f);
    dynamic_y_max = std::min(dynamic_y_max, end[1] + 10.0f);

    double point[4][2], rspoint[2], startpoint[3];
    startpoint[0] = start[0];
    startpoint[1] = start[1];
    startpoint[2] = start[2];

    point[0][0] = cfg_vehicle_length - cfg_wheel_base;
    point[0][1] = cfg_vehicle_width / 2.0;

    point[1][0] = cfg_vehicle_length - cfg_wheel_base;
    point[1][1] = -cfg_vehicle_width / 2.0;

    point[2][0] = -cfg_wheel_base;
    point[2][1] = cfg_vehicle_width / 2.0;

    point[3][0] = -cfg_wheel_base;
    point[3][1] = -cfg_vehicle_width / 2.0;

    for (int i = 0; i < 4; i++)
    {
        Convert(startpoint, point[i], rspoint);
        dynamic_x_min = std::min(dynamic_x_min, rspoint[0]);
        dynamic_x_max = std::max(dynamic_x_max, rspoint[0]);
        dynamic_y_min = std::min(dynamic_y_min, rspoint[1]);
        dynamic_y_max = std::max(dynamic_y_max, rspoint[1]);
    }

    double buffer = 0.1; // 缓冲区
    x_min         = std::floor(dynamic_x_min / resolution) * resolution - buffer;
    x_max         = std::ceil(dynamic_x_max / resolution) * resolution + buffer;
    y_min         = std::floor(dynamic_y_min / resolution) * resolution - buffer;
    y_max         = std::ceil(dynamic_y_max / resolution) * resolution + buffer;

    std::cout << "map size: (" << x_min << ", " << y_min << ") -> (" << x_max << ", "
              << y_max << ")" << std::endl;
    std::cout << "grid resolution: " << resolution << "m" << std::endl;

    // 确保栅格数量 >= 100x100
    x_size  = std::max(100, static_cast<int>(ceil((x_max - x_min) / resolution)) + 1);
    y_size  = std::max(100, static_cast<int>(ceil((y_max - y_min) / resolution)) + 1);
    gridMap = std::vector<std::vector<int>>(y_size, std::vector<int>(x_size, 0));

    saveJsonData();
    return true;
}

// 使用 OpenCV 的抗锯齿绘图函数标记障碍物
void GridMapGenerator::markObstacle(const std::vector<LineSeg_T> &obstacle)
{
    if (obstacle.empty())
    {
        return;
    }
#ifndef ndk
    // 创建 OpenCV 单通道图像，初始全为0
    cv::Mat mat(y_size, x_size, CV_8UC1, cv::Scalar(0));
    // 遍历每两个点进行拟合并绘制直线
    for (size_t i = 0; i < obstacle.size(); i++)
    {
        // 将两个点转换为栅格坐标
        auto [x1, y1] = toGridIdxInt(obstacle[i].pt1.x, obstacle[i].pt1.y);
        auto [x2, y2] = toGridIdxInt(obstacle[i].pt2.x, obstacle[i].pt2.y);

        if (x1 < 0 || x1 >= x_size || y1 < 0 || y1 >= y_size || x2 < 0 || x2 >= x_size ||
            y2 < 0 || y2 >= y_size)
        {
            continue;
        }

        // 用于拟合的点集（只有两个点时，拟合结果就是连接它们的直线）
        std::vector<cv::Point> pts;
        pts.push_back(cv::Point(x1, y1));
        pts.push_back(cv::Point(x2, y2));

        // 使用 cv::fitLine 拟合直线
        cv::Vec4f line;
        cv::fitLine(pts, line, cv::DIST_FAIR, 0, 0.01, 0.01);

        // 此处我们直接使用原始两个点作为直线端点（若需要延长直线，可依据 (vx,vy)
        // 进行延伸）
        cv::Point pt1(x1, y1);
        cv::Point pt2(x2, y2);

        // 绘制直线
        cv::line(mat, pt1, pt2, cv::Scalar(255), 1.8, cv::LINE_AA);
    }

    // 将绘制结果复制回 gridMap
    for (int y = 0; y < y_size; ++y)
    {
        for (int x = 0; x < x_size; ++x)
        {
            gridMap[y][x] = mat.at<uchar>(y, x) > 0 ? 1 : 0;
        }
    }
#endif
}

// 应用形态学操作以平滑障碍物边界
void GridMapGenerator::applyMorphology()
{
#ifndef ndk
    // 将 gridMap 转换为 OpenCV Mat
    cv::Mat mat(y_size, x_size, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < y_size; ++y)
    {
        for (int x = 0; x < x_size; ++x)
        {
            mat.at<uchar>(y, x) = gridMap[y][x] == 1 ? 255 : 0;
        }
    }

    // // 定义膨胀核
    // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    // // 应用膨胀
    // cv::dilate(mat, mat, kernel);

    // // 定义腐蚀核
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    // 应用腐蚀
    cv::erode(mat, mat, kernel);

    // 更新 gridMap
    for (int y = 0; y < y_size; ++y)
    {
        for (int x = 0; x < x_size; ++x)
        {
            gridMap[y][x] = mat.at<uchar>(y, x) > 0 ? 1 : 0;
        }
    }
#endif
}

// 保存栅格地图为 CSV 文件
void GridMapGenerator::saveJsonData()
{
    std::string filename = outputFilePath + "_" + std::to_string(gridId) + "_input.json";
    std::ofstream file(filename);

    // 如果文件成功打开
    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // 写入 CSV 表头
    file << inputJsonData << std::endl;

    file.close();
    // std::cout << "Path saved to " << filename << std::endl;
}

// 生成栅格地图
bool GridMapGenerator::generate(OccupancyGrid &grid)
{
    std::vector<LineSeg_T> obstaclePoints;

    // 解析 JSON 文件
    if (!parseJSON(obstaclePoints))
    {
        std::cerr << "解析 JSON 文件失败。" << std::endl;
        return false;
    }

    // 标记障碍物
    markObstacle(obstaclePoints);

    // 应用形态学操作
    applyMorphology();

    // 填充 OccupancyGrid 信息
    grid.resolution = resolution;
    grid.width      = x_size;
    grid.height     = y_size;
    grid.origin_x   = x_min;
    grid.origin_y   = y_min;

    // 将栅格地图数据填充到 OccupancyGrid 结构中
    grid.data.resize(x_size * y_size);
    for (int y = 0; y < y_size; ++y)
    {
        for (int x = 0; x < x_size; ++x)
        {
            grid.data[y * x_size + x] =
                gridMap[y][x] == 1 ? 100 : 0; // 1 表示障碍物，填充为100
        }
    }

    return true;
}

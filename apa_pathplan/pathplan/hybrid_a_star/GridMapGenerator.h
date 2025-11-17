#ifndef GRIDMAPGENERATOR_H
#define GRIDMAPGENERATOR_H

#include "hybrid_type.h"

class GridMapGenerator
{
  public:
    // 构造函数
    GridMapGenerator(double resolution, double x_min, double x_max, double y_min,
                     double y_max, const std::string &inputJsonPath,
                     const std::string &namePrex);

    // 生成栅格地图
    bool generate(OccupancyGrid &grid);
    const int getGridId() const { return gridId; }

    // 访问器：获取起点和终点
    const std::vector<double> &getStart() const { return start; }
    const std::vector<double> &getEnd() const { return end; }

  private:
    // 栅格地图属性
    double resolution;
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    int x_size;
    int y_size;
    int gridId;

    // 文件路径
    std::string inputJsonData;
    std::string outputFilePath;

    // 存储起点和终点
    std::vector<double> start; // 起点 (curpos)
    std::vector<double> end;   // 终点 (slottarg)

    // 栅格地图矩阵
    std::vector<std::vector<int>> gridMap;

    // 解析 JSON 文件并提取障碍物坐标
    bool parseJSON(std::vector<LineSeg_T> &obstacleLines);

    // 将物理坐标转换为栅格索引
    std::pair<int, int> toGridIdxInt(double x, double y) const;
    std::pair<double, double> toGridIdxDouble(double x, double y) const;

    // 标记障碍物到栅格地图
    void markObstacle(const std::vector<LineSeg_T> &obstacleLines);

    void saveJsonData();

    void applyMorphology();
};

#endif // GRIDMAPGENERATOR_H
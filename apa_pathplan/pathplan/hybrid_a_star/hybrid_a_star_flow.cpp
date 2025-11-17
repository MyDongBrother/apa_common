#include "hybrid_a_star_flow.h"
#include "hybrid_data.h"
#include <opencv2/opencv.hpp>

#ifdef bst
extern void *__dso_handle __attribute__((__visibility__("hidden")));
void *__dso_handle;
#endif

using namespace std;
void SavePathToCSV(const VectorVec5d &path, const std::string &name)
{
    std::string filename = name + "_path.csv";
    std::ofstream file(filename);

    // 如果文件成功打开
    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // 遍历路径，将每个点保存为 CSV 格式
    for (const auto &state : path)
    {
        file << state(0) << "," << state(1) << "," << state(2) << "," << state(3) << ","
             << state(4) << std::endl;
    }

    file.close();
    std::cout << "Path saved to " << filename << std::endl;
}

void SaveTreeToCSV(const VectorVec4d &searched_tree, const std::string &name)
{
    // 动态生成文件名
    std::string filename = name + "_tree.csv";
    std::ofstream file(filename);

    // 如果文件成功打开
    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // 写入 CSV 表头
    file << "x,y,z" << std::endl;

    // 遍历路径，将每个点保存为 CSV 格式
    for (const auto &state : searched_tree)
    {
        file << state.x() << "," << state.y() << ","
             << "0.00" << std::endl;
    }

    file.close();
    std::cout << "TREE saved to " << filename << std::endl;
}

#ifndef ndk
OccupancyGrid loadMapFromImage(const std::string &filename, float resolution,
                               float origin_x, float origin_y)
{
    cv::Mat img = cv::imread(filename, cv::IMREAD_GRAYSCALE);
    if (img.empty())
    {
        throw std::runtime_error("Failed to load image: " + filename);
    }

    OccupancyGrid grid;

    grid.resolution = resolution;
    grid.origin_x   = origin_x;
    grid.origin_y   = origin_y;
    grid.width      = img.cols;
    grid.height     = img.rows;
    grid.data.resize(grid.width * grid.height);

    for (int y = 0; y < img.rows; y++)
    {
        for (int x = 0; x < img.cols; x++)
        {
            int idx     = y * grid.width + x;
            uchar pixel = img.at<uchar>(y, x);
            if (pixel == 255)
            { // 白色 (空闲)
                grid.data[idx] = 0;
            }
            else if (pixel == 0)
            { // 黑色 (占用)
                grid.data[idx] = 100;
            }
            else
            { // 灰色 (未知)
                grid.data[idx] = -1;
            }
        }
    }

    return grid;
}
#endif

double Mod2Pi(const double &x)
{
    double v = fmod(x, 2 * M_PI);

    if (v < -M_PI)
    {
        v += 2.0 * M_PI;
    }
    else if (v > M_PI)
    {
        v -= 2.0 * M_PI;
    }

    return v;
}

VectorVec5d HybridAStarFlow::result_path;
HybridAStarFlow::HybridAStarFlow()
{
    result_path.clear();
    kinodynamic_astar_searcher_ptr_ = std::make_shared<HybridAStar>(
        cfg_steering_angle, cfg_steering_angle_discrete_num, cfg_segment_length,
        cfg_segment_length_discrete_num, cfg_wheel_base, cfg_steering_penalty,
        cfg_reversing_penalty, cfg_steering_change_penalty, cfg_shot_distance,
        cfg_grid_size_phi);
}

VectorVec5d *HybridAStarFlow::Run(OccupancyGrid &grid, float map_resolution,
                                  float grid_resolution, Vec3d start_state,
                                  Vec3d goal_state, const std::string &outputFilePath)
{
    std::cout << "grid_resolution: " << grid_resolution << std::endl;
    std::cout << "map_resolution: " << map_resolution << std::endl;
    std::cout << "grid.origin_x: " << grid.origin_x << std::endl;
    std::cout << "grid.width: " << grid.width << std::endl;
    std::cout << "grid.origin_y: " << grid.origin_y << std::endl;
    std::cout << "grid.height: " << grid.height << std::endl;

    // print the start_state and goal_state
    std::cout << "start_state: " << start_state.x() << " " << start_state.y() << " "
              << start_state.z() << std::endl;
    std::cout << "goal_state: " << goal_state.x() << " " << goal_state.y() << " "
              << goal_state.z() << std::endl;

    result_path.clear();
    kinodynamic_astar_searcher_ptr_->Init(grid.origin_x, grid.width, grid.origin_y,
                                          grid.height, grid_resolution, map_resolution);

    for (int y = 0; y < grid.height; ++y)
    {
        for (int x = 0; x < grid.width; ++x)
        {
            if (grid.data[y * grid.width + x] == 100)
            {
                kinodynamic_astar_searcher_ptr_->SetObstacle(
                    static_cast<unsigned int>(x), static_cast<unsigned int>(y));
            }
        }
    }

    if (kinodynamic_astar_searcher_ptr_->Search(start_state, goal_state))
    {
        kinodynamic_astar_searcher_ptr_->GetPath(result_path);
        SavePathToCSV(result_path, outputFilePath);

        // auto searchedTree = kinodynamic_astar_searcher_ptr_->GetSearchedTree();
        // SaveTreeToCSV(searchedTree, "tree_output-");
    }
    else
    {
        // auto searchedTree = kinodynamic_astar_searcher_ptr_->GetSearchedTree();
        // SaveTreeToCSV(searchedTree, "tree_output_failure");
        // cout << "path is not found" << endl;
    }

    kinodynamic_astar_searcher_ptr_->Reset();
    return &result_path;
}

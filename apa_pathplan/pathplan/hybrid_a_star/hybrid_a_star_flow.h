#ifndef HYBRID_A_STAR_FLOW_H
#define HYBRID_A_STAR_FLOW_H

#include "hybrid_a_star.h"
#include "hybrid_type.h"

// HybridAStarFlow 类定义
class HybridAStarFlow
{
  public:
    HybridAStarFlow();

    VectorVec5d *Run(OccupancyGrid &grid, float map_resolution, float grid_resolution,
                     Vec3d start_state, Vec3d goal_state,
                     const std::string &outputFilePath);

  private:
    std::shared_ptr<HybridAStar> kinodynamic_astar_searcher_ptr_;

    std::string map_file;

    static VectorVec5d result_path;

    double origin_x;

    double origin_y;

    Vec3d start_state;

    Vec3d goal_state;
};

#endif // HYBRID_A_STAR_FLOW_H

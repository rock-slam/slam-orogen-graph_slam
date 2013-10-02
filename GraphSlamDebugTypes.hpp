#ifndef GRAPH_SLAM_DEBUG_TASK_TYPES_HPP_
#define GRAPH_SLAM_DEBUG_TASK_TYPES_HPP_

#include <vector>
#include <string>
#include <base/time.h>

namespace graph_slam
{

struct VelodyneSlamDebug
{
    base::Time time;
    std::string graphviz;
    int graph_num_vertices;
    int graph_num_edges;
    double graph_chi2_error;
    double graph_optimization_time;
    double remove_vertices_time;
    double find_edge_candidates_time;
    double update_environment_time;
    double try_edge_candidate_time;
    VelodyneSlamDebug()
    : time(base::Time::now()), graph_num_vertices(0), graph_num_edges(0), 
      graph_chi2_error(0.0), graph_optimization_time(0.0), remove_vertices_time(0.0),
      find_edge_candidates_time(0.0), update_environment_time(0.0), try_edge_candidate_time(0.0) {}
};

}

#endif

/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "VelodyneSLAM.hpp"

#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <graph_slam/vertex_se3_gicp.hpp>
#include <graph_slam/edge_se3_gicp.hpp>
#include <velodyne_lidar/pointcloudConvertHelper.hpp>
#include <graph_slam/matrix_helper.hpp>


using namespace graph_slam;

VelodyneSLAM::VelodyneSLAM(std::string const& name)
    : VelodyneSLAMBase(name)
{
}

VelodyneSLAM::VelodyneSLAM(std::string const& name, RTT::ExecutionEngine* engine)
    : VelodyneSLAMBase(name, engine)
{
}

VelodyneSLAM::~VelodyneSLAM()
{
}

void VelodyneSLAM::handleLidarData(const base::Time &ts, bool use_simulated_data)
{    
    // get static transformation
    Eigen::Affine3d laser2body;
    if (!_laser2body.get(ts, laser2body))
    {
        std::cerr << "skip, have no laser2body transformation sample!" << std::endl;
        return;
    }
    
    // set a odometry covariance if it is nan
    if(is_nan(body2odometry.getCovariance()))
        body2odometry.setCovariance(100 * envire::TransformWithUncertainty::Covariance::Identity());
    
    // reset number of edge candidates handled in the update hook
    try_edges_on_update = 1;

    Eigen::Affine3d odometry_delta = last_odometry_transformation.getTransform().inverse() * body2odometry.getTransform();
    if(odometry_delta.translation().norm() > _vertex_distance.get() || optimizer.vertices().size() == 0)
    {
        try
        {
            // fill envire pointcloud
            std::vector<Eigen::Vector3d> pointcloud;
            if(use_simulated_data)
            {
                // transform pointcloud to body frame
                for(std::vector<base::Vector3d>::const_iterator it = new_simulated_pointcloud_sample.points.begin(); it != new_simulated_pointcloud_sample.points.end(); it++)
                {
                    pointcloud.push_back(laser2body * (*it));
                }
            }
            else
            {
                // filter point cloud
                velodyne_lidar::MultilevelLaserScan filtered_lidar_sample;
                velodyne_lidar::ConvertHelper::filterOutliers(new_lidar_sample, filtered_lidar_sample, _maximum_angle_to_neighbor, _minimum_valid_neighbors);
                
                // add new vertex to graph
                velodyne_lidar::ConvertHelper::convertScanToPointCloud(filtered_lidar_sample, pointcloud, laser2body);
            }
            
            // add new vertex
            if(!optimizer.addVertex(body2odometry, pointcloud))
                throw std::runtime_error("failed to add a new vertex");

            new_vertecies++;
        }
        catch(std::runtime_error e)
        {
            std::cerr << "Exception while handling lidar sample: " << e.what() << std::endl;
        }
        
        last_odometry_transformation = body2odometry;
    }

    // optimization
    unsigned new_edges = optimizer.edges().size() - edge_count;
    if(new_edges >= _run_graph_optimization_counter)
    {
        edge_count = optimizer.edges().size();

        // run graph optimization
        if(optimizer.optimize(2) < 1)
            throw std::runtime_error("optimization failed");

        if(new_vertecies >= 5)
        {
            new_vertecies = 0;

            // remove old vertecies
            optimizer.removeVerticesFromGrid();
            
            // find new edges
            optimizer.findEdgeCandidates();

            // update envire
            if(!optimizer.updateEnvire())
                throw std::runtime_error("can't update envire transformations and maps for one or more vertecies");
        }
    }
}

void VelodyneSLAM::lidar_samplesTransformerCallback(const base::Time &ts, const ::velodyne_lidar::MultilevelLaserScan &lidar_sample)
{
    new_lidar_sample = lidar_sample;
    // get dynamic transformation
    if (!_body2odometry.get(ts, body2odometry, true))
    {
        std::cerr << "skip, have no body2odometry transformation sample!" << std::endl;
        return;
    }
    handleLidarData(lidar_sample.time, false);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See VelodyneSLAM.hpp for more detailed
// documentation about them.

bool VelodyneSLAM::configureHook()
{
    if (! VelodyneSLAMBase::configureHook())
        return false;
    
    new_vertecies = 0;
    edge_count = 0;
    try_edges_on_update = 0;
    last_envire_update.microseconds = 0;
    last_odometry_transformation = envire::TransformWithUncertainty::Identity();
    optimizer.setMLSMapConfiguration(_use_mls, _grid_size_x, _grid_size_y, _cell_resolution_x, _cell_resolution_y);
    event_filter.reset(new MLSGridEventFilter());
    
    g2o::OptimizableGraph::initMultiThreading();
    
    // set gicp config
    graph_slam::GICPConfiguration gicp_config;
    gicp_config.max_sensor_distance = _max_icp_distance;
    gicp_config.max_fitness_score = _max_icp_fitness_score;
    optimizer.updateGICPConfiguration(gicp_config);
    
    // enable debug output
    optimizer.setVerbose(true);

    optimizer.setupMaxVertexGrid(_max_vertices_per_cell, _grid_size_x, _grid_size_y, _vertex_grid_cell_resolution);
    
    return true;
}
bool VelodyneSLAM::startHook()
{
    if (! VelodyneSLAMBase::startHook())
        return false;
    
    
    
    return true;
}
void VelodyneSLAM::updateHook()
{
    if( orocos_emitter.use_count() == 0 && _envire_map.connected() )
    {
        // register the binary event dispatcher, 
        // which will write envire data to a port
        boost::shared_ptr<envire::Environment> env = optimizer.getEnvironment();
        orocos_emitter.reset(new envire::OrocosEmitter( _envire_map ));
        orocos_emitter->useContextUpdates( env.get() );
        orocos_emitter->useEventQueue( true );
        orocos_emitter->attach( env.get() );
        if(_use_mls)
            orocos_emitter->setFilter(event_filter.get());
    }
    
    VelodyneSLAMBase::updateHook();
    
    // write adjusted odometry pose
    base::samples::RigidBodyState odometry_pose;
    if(_odometry_samples.readNewest(odometry_pose) == RTT::NewData) 
    {
        base::samples::RigidBodyState adjusted_odometry_pose;
        if(optimizer.adjustOdometryPose(odometry_pose, adjusted_odometry_pose))
            _pose_samples.write(adjusted_odometry_pose);
    }
    
    // write envire updates
    if( orocos_emitter.use_count() > 0 )
    {
        if( _envire_map.connected() )
        {
            if( (last_envire_update + base::Time::fromSeconds(_envire_period.value())) < base::Time::now() ) 
            {
                orocos_emitter->flush();
                last_envire_update = base::Time::now();
            }
        }
        else
        {
            orocos_emitter.reset();
        }
    }
    
    // handle simulated data
    if(_simulated_pointcloud.readNewest(new_simulated_pointcloud_sample) == RTT::NewData)
    {
        body2odometry.setTransform(odometry_pose.getTransform());
        body2odometry.setCovariance(combineToPoseCovariance(odometry_pose.cov_position, odometry_pose.cov_orientation));
        if(new_simulated_pointcloud_sample.points.size() > 0)
            handleLidarData(new_simulated_pointcloud_sample.time, true);
    }
    
    // create new edges
    if(try_edges_on_update > 0)
    {
        optimizer.tryBestEdgeCandidates(1);
        try_edges_on_update--;
    }
}
void VelodyneSLAM::errorHook()
{
    VelodyneSLAMBase::errorHook();
}
void VelodyneSLAM::stopHook()
{
    VelodyneSLAMBase::stopHook();
    
    if(_environment_debug_path.get() != "")
    {
        // write environment
        optimizer.getEnvironment()->serialize(_environment_debug_path.get());

        // write graph viz
        std::filebuf fb;
        std::string full_path(_environment_debug_path.get());
        full_path += "/graph_viz.dot";
        fb.open (full_path.c_str(),std::ios::out);
        std::ostream os(&fb);
        optimizer.dumpGraphViz(os);
        fb.close();
    }
}
void VelodyneSLAM::cleanupHook()
{
    VelodyneSLAMBase::cleanupHook();
    
    orocos_emitter.reset();
    
    // freeing the graph memory
    optimizer.clear();

    // destroy all the singletons
    g2o::Factory::destroy();
    g2o::OptimizationAlgorithmFactory::destroy();
    g2o::HyperGraphActionLibrary::destroy();
}

/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "VelodyneSLAM.hpp"
#include <GraphSlamDebugTypes.hpp>

#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <graph_slam/vertex_se3_gicp.hpp>
#include <graph_slam/edge_se3_gicp.hpp>
#include <graph_slam/graph_slam_config.hpp>
#include <velodyne_lidar/pointcloudConvertHelper.hpp>
#include <graph_slam/matrix_helper.hpp>
#include <graph_slam/pointcloud_helper.hpp>


#ifndef MEASURE_TIME
#define MEASURE_TIME(exp, time_in_s) \
{ \
    clock_t _clock_time_ = clock(); \
    exp; \
    time_in_s = (double)(clock() - _clock_time_) / (double)CLOCKS_PER_SEC; \
}
#endif

using namespace graph_slam;

VelodyneSLAM::VelodyneSLAM(std::string const& name)
    : VelodyneSLAMBase(name), map_updated(false)
{
}

VelodyneSLAM::VelodyneSLAM(std::string const& name, RTT::ExecutionEngine* engine)
    : VelodyneSLAMBase(name, engine), map_updated(false)
{
}

VelodyneSLAM::~VelodyneSLAM()
{
}

void VelodyneSLAM::handleLidarData(const base::Time &ts, const ::base::samples::DepthMap* lidar_sample,
                                   const base::samples::Pointcloud* simulated_pointcloud_sample,
                                   const ::velodyne_lidar::MultilevelLaserScan* lidar_sample_deprecated)
{
    new_state = RUNNING;
    last_sample_time = ts;

    // get transformations
    Eigen::Affine3d laser2body;
    if (!_laser2body.get(ts, laser2body))
    {
        RTT::log(RTT::Error) << "skip, have no laser2body transformation sample!" << RTT::endlog();
        new_state = MISSING_TRANSFORMATION;
        return;
    }
    envire::TransformWithUncertainty body2odometry;
    if (!_body2odometry.get(ts, body2odometry, true))
    {
        RTT::log(RTT::Error) << "skip, have no body2odometry transformation sample!" << RTT::endlog();
        new_state = MISSING_TRANSFORMATION;
	last_body2odometry_valid = false;
        return;
    }
    
    // set a odometry covariance if it is nan
    if(is_nan(body2odometry.getCovariance()))
        body2odometry.setCovariance(100 * envire::TransformWithUncertainty::Covariance::Identity());
    
    // reset number of edge candidates handled in the update hook
    try_edges_on_update = 1;

    Eigen::Affine3d odometry_delta = last_vertex_odometry_transformation.getTransform().inverse() * body2odometry.getTransform();
    if( integrate_new_samples && last_body2odometry_valid && (odometry_delta.translation().norm() > _vertex_distance.get() || (base::Time::now() - last_new_vertex).toSeconds() > _new_vertex_time) )
    {
        try
        {
            // fill envire pointcloud
            std::vector<Eigen::Vector3d> pointcloud;
            if(lidar_sample)
            {
                // filter point cloud
                base::samples::DepthMap filtered_lidar_sample = *lidar_sample;
                filterMinDistance(filtered_lidar_sample, 1.0);
                filterOutliers(filtered_lidar_sample, _maximum_angle_to_neighbor, _minimum_valid_neighbors);

                // convert scan to pointcloud
		Eigen::Affine3d scanBegin2scanEnd = last_body2odometry.getTransform().inverse() * body2odometry.getTransform();
                filtered_lidar_sample.convertDepthMapToPointCloud(pointcloud, scanBegin2scanEnd.inverse() * laser2body, laser2body, true, true, false);
            }
            else if(simulated_pointcloud_sample)
            {
                // transform pointcloud to body frame
                for(std::vector<base::Vector3d>::const_iterator it = simulated_pointcloud_sample->points.begin(); it != simulated_pointcloud_sample->points.end(); it++)
                {
                    pointcloud.push_back(laser2body * (*it));
                }
            }
            else if(lidar_sample_deprecated)
            {
                // filter point cloud
                velodyne_lidar::MultilevelLaserScan filtered_lidar_sample;
                velodyne_lidar::ConvertHelper::filterOutliers(*lidar_sample_deprecated, filtered_lidar_sample, _maximum_angle_to_neighbor, _minimum_valid_neighbors);

                // convert scan to pointcloud
                Eigen::Affine3d scanBegin2scanEnd = last_body2odometry.getTransform().inverse() * body2odometry.getTransform();
                velodyne_lidar::ConvertHelper::convertScanToPointCloud(filtered_lidar_sample, pointcloud, scanBegin2scanEnd.inverse() * laser2body, laser2body);
            }
            else
            {
                throw std::runtime_error("data sample is missing");
            }

            if(pointcloud.empty())
                throw std::runtime_error("pointcloud is empty");
            
            // add new vertex to graph
            if(!optimizer.addVertex(body2odometry, pointcloud, laser2body))
                throw std::runtime_error("failed to add a new vertex");

            new_vertecies++;
        }
        catch(const std::runtime_error & e)
        {
            RTT::log(RTT::Error) << "Exception while adding new lidar sample: " << e.what() << RTT::endlog();
            new_state = ADD_VERTEX_FAILED;
        }
        
        last_new_vertex = base::Time::now();
        last_vertex_odometry_transformation = body2odometry;
    }
    
    last_body2odometry = body2odometry;
    last_body2odometry_valid = true;

    // optimization
    int new_edges = optimizer.edges().size() - edge_count;
    if(new_edges >= _run_graph_optimization_counter || (!initial_optimization && optimizer.vertices().size() >= 2))
    {
        edge_count = optimizer.edges().size();

        try
        {
            // run graph optimization
            if(optimizer.optimize(2) < 1)
                throw std::runtime_error("optimization failed!");
            initial_optimization = true;
            if(_enable_debug)
                writeOptimizerDebugInformation();

            if(int(new_vertecies) >= _run_icp_candidate_search_counter.value())
            {
                new_vertecies = 0;

                // remove old vertices
                MEASURE_TIME(optimizer.removeVerticesFromGrid(), debug_information.remove_vertices_time);
                
                // find new edges
                MEASURE_TIME(optimizer.findEdgeCandidates(), debug_information.find_edge_candidates_time);
            }
        }
        catch(const std::runtime_error & e)
        {
            RTT::log(RTT::Error) << "Exception while optimizing graph: " << e.what() << RTT::endlog();
            new_state = GRAPH_OPTIMIZATION_FAILED;
        }
    }
}

void VelodyneSLAM::lidar_samplesTransformerCallback(const base::Time &ts, const ::base::samples::DepthMap &lidar_sample)
{
    handleLidarData(lidar_sample.time, &lidar_sample);
}

void VelodyneSLAM::lidar_samples_deprecatedTransformerCallback(const base::Time &ts,
                                                               const ::velodyne_lidar::MultilevelLaserScan &lidar_sample_deprecated)
{
    handleLidarData(lidar_sample_deprecated.time, NULL, NULL, &lidar_sample_deprecated);
}

void VelodyneSLAM::simulated_pointcloudTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &simulated_pointcloud_sample)
{
    // check simulated pointcloud for nan values
    base::samples::Pointcloud simulated_pointcloud;
    simulated_pointcloud.time = simulated_pointcloud_sample.time;
    for(unsigned i = 0; i < simulated_pointcloud_sample.points.size(); i++)
    {
        if(!simulated_pointcloud_sample.points[i].hasNaN())
        {
            simulated_pointcloud.points.push_back(simulated_pointcloud_sample.points[i]);
        }
    }

    handleLidarData(simulated_pointcloud_sample.time, NULL, &simulated_pointcloud);
}

bool VelodyneSLAM::generateMap()
{
    bool err = false;
    try
    {
        // run graph optimization
        if(optimizer.activeVertices().size() && optimizer.optimize(5) < 1)
        {
            err = true;
            RTT::log(RTT::Error) << "optimization failed" << RTT::endlog();
            new_state = GRAPH_OPTIMIZATION_FAILED;
        }
        if(_enable_debug)
            writeOptimizerDebugInformation();
        
        // update envire
        MEASURE_TIME
        (
            if(!optimizer.updateEnvire())
            {
                err = true;
                RTT::log(RTT::Error) << "environment update failed" << RTT::endlog();
                new_state = MAP_GENERATION_FAILED;
            } else {
                map_updated = true;
            },
            debug_information.update_environment_time
        );
    }
    catch(const std::runtime_error & e)
    {
        RTT::log(RTT::Error) << "Exception while generating MLS map: " << e.what() << RTT::endlog();
        new_state = MAP_GENERATION_FAILED;
    }
    return !err;
}

bool VelodyneSLAM::saveEnvironment(::std::string const & path)
{
    try
    {
        // generate map update
        generateMap();

        // serialize environment
        if(path.empty() && !_environment_debug_path.get().empty())
            optimizer.getEnvironment()->serialize(_environment_debug_path.get());
        else
            optimizer.getEnvironment()->serialize(path);
        return true;
    }
    catch(const std::runtime_error & e)
    {
        RTT::log(RTT::Error) << "Exception while serialization of the map data: " << e.what() << RTT::endlog();
    }
    return false;
}

bool VelodyneSLAM::activateLidarIntegration(bool b)
{
    integrate_new_samples = b;
    if(integrate_new_samples)
        RTT::log(RTT::Warning) << "Integration of new samples is activated." << RTT::endlog();
    else
        RTT::log(RTT::Warning) << "Note: Integration of new samples is deactivated!" << RTT::endlog();
    return true;
}

void VelodyneSLAM::writeOptimizerDebugInformation()
{
    const g2o::BatchStatisticsContainer& stats = optimizer.batchStatistics();
    if(!stats.empty())
    {
        debug_information.graph_num_vertices = stats[0].numVertices;
        debug_information.graph_num_edges = stats[0].numEdges;
        debug_information.graph_chi2_error = stats[stats.size()-1].chi2;
        debug_information.graph_optimization_time = stats[0].timeIteration;
    }

    std::stringstream stream;
    optimizer.dumpGraphViz(stream);
    debug_information.graphviz = stream.str();
}

void VelodyneSLAM::body2OdometryTransformerCallback(const base::Time& ts, const transformer::Transformation &tr)
{
    
    base::samples::RigidBodyState adjusted_odometry_pose, odometry_pose;
    if(!tr.get(ts, odometry_pose))
    {
        throw std::runtime_error("Internal Error");
    }
    if(optimizer.adjustOdometryPose(odometry_pose, adjusted_odometry_pose))
    {
        adjusted_odometry_pose.sourceFrame = _body_frame.get();
        adjusted_odometry_pose.targetFrame = _world_frame.get();
        adjusted_odometry_pose.time = ts;
        _pose_samples.write(adjusted_odometry_pose);
        
        graph_slam::PoseProviderUpdate update;
        update.body2odometry = odometry_pose.getPose();
        update.body2world = adjusted_odometry_pose.getPose();
        update.time = adjusted_odometry_pose.time;
        _pose_provider_update.write(update);
    }
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See VelodyneSLAM.hpp for more detailed
// documentation about them.

bool VelodyneSLAM::configureHook()
{
    if (! VelodyneSLAMBase::configureHook())
        return false;

    // get static transformations
    Eigen::Affine3d map2world = Eigen::Affine3d::Identity();
    map2world.translation() = _grid_offset.get();
    Eigen::Affine3d start_pose = Eigen::Affine3d::Identity();
    if(_start_pose.get().hasValidPosition() && _start_pose.get().hasValidOrientation())
    {
        start_pose = _start_pose.get().getTransform();
    }
    
    // setup initial configuration
    last_state = PRE_OPERATIONAL;
    new_state = RUNNING;
    initial_optimization = false;
    map_updated = false;
    last_body2odometry_valid = false;
    integrate_new_samples = true;
    new_vertecies = 0;
    edge_count = 0;
    try_edges_on_update = 0;
    last_new_vertex.microseconds = 0;
    last_envire_update.microseconds = 0;
    last_sample_time.microseconds = 0;
    debug_information.time.microseconds = 0;
    debug_information = VelodyneSlamDebug();
    last_vertex_odometry_transformation = envire::TransformWithUncertainty::Identity();
    last_body2odometry = envire::TransformWithUncertainty::Identity();
    optimizer.setMLSMapConfiguration(_use_mls, _mls_config.get(), "/mls-grid", _grid_size_x, _grid_size_y, _cell_resolution_x, _cell_resolution_y, _grid_min_z, _grid_max_z);
    optimizer.setMap2WorldTransformation(Eigen::Isometry3d(map2world.matrix()));
    optimizer.setRobotStart2WorldTransformation(Eigen::Isometry3d(start_pose.matrix()));
    event_filter.reset(new MLSGridEventFilter());
    optimizer.setInitialFootprintRadius(_footprint_radius.value());
    
    g2o::OptimizableGraph::initMultiThreading();
    
    // set gicp config
    graph_slam::GICPConfiguration gicp_config;
    gicp_config.max_sensor_distance = _max_icp_distance;
    gicp_config.max_fitness_score = _max_icp_fitness_score;
    optimizer.updateGICPConfiguration(gicp_config);

    optimizer.setupMaxVertexGrid(_max_vertices_per_cell, _grid_size_x, _grid_size_y, _vertex_grid_cell_resolution);

    // load reference map
    if(_apriori_map.get() != "")
    {
        try
        {
            boost::shared_ptr<envire::Environment> env(envire::Environment::unserialize(_apriori_map.get()));

            if(!optimizer.setAPrioriMap(env))
                RTT::log(RTT::Error) << "Failed to add a-priori map." << RTT::endlog();
            else //if(!_start_pose.get().hasValidPosition() || !_start_pose.get().hasValidOrientation())
            {
                RTT::log(RTT::Info) << "Successfully loaded a-priori map." << RTT::endlog();

                if(_determine_start_pose_from_apriori_map.value())
                {
                    // set last vertex pose as start pose
                    RTT::log(RTT::Info) << "Load pose of last pointcloud as start pose." << RTT::endlog();
                    std::vector<envire::Pointcloud*> pointclouds = env->getItems<envire::Pointcloud>();
                    if(!pointclouds.empty())
                    {
                        // find pointcloud with the highest id
                        envire::Pointcloud* pointcloud = 0;
                        long pc_id = pointclouds.front()->getUniqueIdNumericalSuffix();
                        for(unsigned i = 1; i < pointclouds.size(); i++)
                        {
                            long id = pointclouds[i]->getUniqueIdNumericalSuffix();
                            if(id > pc_id)
                            {
                                pointcloud = pointclouds[i];
                                pc_id = id;
                            }
                        }
                        if(pointcloud)
                        {
                            base::Transform3d pc2world = env->relativeTransform(pointcloud->getFrameNode(), env->getRootNode());
                            optimizer.setRobotStart2WorldTransformation(Eigen::Isometry3d(pc2world.matrix()));
                        }
                    }

                }
            }
        }
        catch(const std::runtime_error & e)
        {
            RTT::log(RTT::Error) << "Failed to load a-priori map: " << e.what() << RTT::endlog();
        }
    }

    _transformer.registerTransformCallback(_body2odometry, boost::bind(&VelodyneSLAM::body2OdometryTransformerCallback, this, _1, _2));

    
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
        if(_use_mls)
            orocos_emitter->setFilter(event_filter.get());
        orocos_emitter->useEventQueue( true );
        orocos_emitter->useContextUpdates( env.get() );
        orocos_emitter->attach( env.get() );
    }

    // enable debug output
    if(_enable_debug)
        optimizer.setComputeBatchStatistics(true);
    
    VelodyneSLAMBase::updateHook();
    
    // write envire updates
    if( orocos_emitter.use_count() > 0 )
    {
        if( _envire_map.connected() )
        {
            if( (last_envire_update + base::Time::fromSeconds(_envire_period.value())) < base::Time::now() &&
                    map_updated ) 
            {
                orocos_emitter->setTime(last_sample_time);
                orocos_emitter->flush();
                map_updated = false;
                last_envire_update = base::Time::now();
            }
        }
        else
        {
            orocos_emitter.reset();
        }
    }
    
    // create new edges
    if(try_edges_on_update > 0)
    {
        try
        {
            MEASURE_TIME(optimizer.tryBestEdgeCandidates(1), debug_information.try_edge_candidate_time);
        }
        catch(const std::runtime_error & e)
        {
            RTT::log(RTT::Error) << "Failed to add new edge from candidates: " << e.what() << RTT::endlog();
        }
        try_edges_on_update--;
    }

    // write debug information
    if(_enable_debug && debug_information.time + base::Time::fromSeconds(1.0) <  base::Time::now())
    {
        debug_information.time = base::Time::now();
        _debug_information.write(debug_information);
    }

    // write state if it has changed
    if(last_state != new_state)
    {
        last_state = new_state;
        state(new_state);
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
        generateMap();
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
    event_filter.reset();
    
    // freeing the graph memory
    optimizer.clear();

    // destroy all the singletons
    g2o::Factory::destroy();
    g2o::OptimizationAlgorithmFactory::destroy();
    g2o::HyperGraphActionLibrary::destroy();
}

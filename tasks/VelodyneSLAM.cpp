/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "VelodyneSLAM.hpp"

#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <graph_slam/vertex_se3_gicp.hpp>
#include <graph_slam/edge_se3_gicp.hpp>
#include <velodyne_lidar/pointcloudConvertHelper.hpp>
#include <envire/maps/Pointcloud.hpp>
#include <envire/maps/MLSGrid.hpp>
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

void VelodyneSLAM::lidar_samplesTransformerCallback(const base::Time &ts, const ::velodyne_lidar::MultilevelLaserScan &lidar_sample)
{
    // get transformation
    Eigen::Affine3d laser2body;
    if (!_laser2body.get(lidar_sample.time, laser2body))
    {
        std::cerr << "skip, have no laser2body transformation sample!" << std::endl;
        return;
    }
    envire::TransformWithUncertainty body2odometry;
    if (!_body2odometry.get(lidar_sample.time, body2odometry, true))
    {
        std::cerr << "skip, have no body2odometry transformation sample!" << std::endl;
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
        // add point cloud to envire
        envire::Pointcloud* envire_pointcloud = new envire::Pointcloud();
        envire::FrameNode* frame = new envire::FrameNode();
        env->addChild(env->getRootNode(), frame);
        env->setFrameNode(envire_pointcloud, frame);
        if(use_mls)
            env->addInput(projection.get(), envire_pointcloud);
        
        try
        {
            // add new vertex to graph
            velodyne_lidar::ConvertHelper::convertScanToPointCloud(lidar_sample, envire_pointcloud->vertices, laser2body);
            if(!optimizer.addVertex(body2odometry, envire_pointcloud))
                throw std::runtime_error("failed to add a new vertex");
            
            // run optimization
            if(optimizer.vertices().size() % 5 == 0)
            {             
                if(optimizer.optimize(5) < 1)
                    throw std::runtime_error("optimization failed");
                
                // find new edges
                optimizer.findEdgeCandidates();
                // update envire
                if(!optimizer.updateEnvireTransformations())
                    throw std::runtime_error("can't update envire transformations for one or more vertecies");
                if(use_mls)
                    projection->updateAll();
            }
            else
            {
                // update envire
                if(!optimizer.updateEnvireTransformations())
                    throw std::runtime_error("can't update envire transformations for one or more vertecies");
            }
        }
        catch(std::runtime_error e)
        {
            std::cerr << "Exception while handling lidar sample: " << e.what() << std::endl;
        }
        
        last_odometry_transformation = body2odometry;
    }
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See VelodyneSLAM.hpp for more detailed
// documentation about them.

bool VelodyneSLAM::configureHook()
{
    if (! VelodyneSLAMBase::configureHook())
        return false;
    
    try_edges_on_update = 0;
    last_envire_update.microseconds = 0;
    last_odometry_transformation = envire::TransformWithUncertainty::Identity();
    env.reset(new envire::Environment());
    use_mls = _use_mls;
    
    g2o::OptimizableGraph::initMultiThreading();
    
    // set gicp config
    graph_slam::GICPConfiguration gicp_config;
    gicp_config.max_sensor_distance = 1.5;
    optimizer.updateGICPConfiguration(gicp_config);
    
    // enable debug output
    optimizer.setVerbose(true);
    
    if(use_mls)
    {
        // setup envire mls grid
        double grid_count_x = _grid_size_x / _cell_resolution_x;
        double grid_count_y = _grid_size_y / _cell_resolution_y;
        envire::MultiLevelSurfaceGrid* mls = new envire::MultiLevelSurfaceGrid(grid_count_y, grid_count_x, _cell_resolution_x, _cell_resolution_y, -0.5 * _grid_size_x, -0.5 * _grid_size_y);
        projection.reset(new envire::MLSProjection());
        
        env->attachItem(mls);
        envire::FrameNode *fn = new envire::FrameNode();
        env->getRootNode()->addChild(fn);
        mls->setFrameNode(fn);
        env->addOutput(projection.get(), mls);
    }
    
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
        orocos_emitter.reset(new envire::OrocosEmitter( _envire_map ));
        orocos_emitter->useContextUpdates( env.get() );
        orocos_emitter->useEventQueue( true );
        orocos_emitter->attach( env.get() );
    }
    
    VelodyneSLAMBase::updateHook();
    
    // write adjusted odometry pose
    base::samples::RigidBodyState odometry_pose;
    if(_odometry_samples.read(odometry_pose) == RTT::NewData) 
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
    
    // create new edges
    if(try_edges_on_update > 0)
    {
        optimizer.tryBestEdgeCandidate();
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
        env->serialize(_environment_debug_path.get());
    
}
void VelodyneSLAM::cleanupHook()
{
    VelodyneSLAMBase::cleanupHook();
    
    orocos_emitter.reset();
    
    env.reset();
    
    // freeing the graph memory
    optimizer.clear();

    // destroy all the singletons
    g2o::Factory::destroy();
    g2o::OptimizationAlgorithmFactory::destroy();
    g2o::HyperGraphActionLibrary::destroy();
}

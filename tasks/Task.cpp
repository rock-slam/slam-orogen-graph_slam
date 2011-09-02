#include "Task.hpp"
#include <vizkit/EnvireVisualization.hpp>

#include <envire/tools/GraphViz.hpp>

#include "PoseGraph.hpp"

using namespace graph_slam;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state), env(NULL), firstNode(true)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TaskBase(name, engine, initial_state), env(NULL), firstNode( true )
{
}

Task::~Task()
{
}

void Task::distance_framesTransformerCallback(const base::Time &ts, const ::base::samples::DistanceImage &distance_frames_sample)
{
    // get the current transforms
    Eigen::Affine3d body2odometry, lcamera2body;
    if( !_body2odometry.get( ts, body2odometry ) || !_lcamera2body.get( ts, lcamera2body ) )
	return;

    std::cerr << "add node" << std::endl;
    std::cerr << body2odometry.translation().transpose() << std::endl;

    // get the transformwithuncertainty
    // TODO get the covariance from the transformer module
    envire::TransformWithUncertainty body2odometryTU( 
	    body2odometry, Eigen::Matrix<double,6,6>::Identity() );

    envire::TransformWithUncertainty body2bodyPrev;
    if( firstNode )
	body2bodyPrev = envire::TransformWithUncertainty( Eigen::Affine3d::Identity(), Eigen::Matrix<double,6,6>::Identity() );
    else
	body2bodyPrev = prevBody2Odometry.inverse() * body2odometryTU;

    // initialize a new node, and add the sensor readings to it
    graph->initNode( body2bodyPrev );
    graph->addSensorReading( distance_frames_sample, lcamera2body );
    graph->addNode();

    std::cerr << "add node done." << std::endl;

    firstNode = false;
    prevBody2Odometry = body2odometryTU;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    env = new envire::Environment();
    if( _debug_viz.value() )
    {
	viz.start();
	viz.getWidget()->updateData( env );
    }

    graph = new PoseGraph( env );

    return true;
}

// bool Task::startHook()
// {
//     if (! TaskBase::startHook())
//         return false;
//     return true;
// }

void Task::updateHook()
{
    TaskBase::updateHook();
}

// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }
// void Task::stopHook()
// {
//     TaskBase::stopHook();
// }

void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    delete graph;
    delete env;
}


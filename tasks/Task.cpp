/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <envire/Core.hpp>
#include <vizkit/EnvireVisualization.hpp>

using namespace graph_slam;
using namespace envire;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state), env(NULL)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TaskBase(name, engine, initial_state), env(NULL)
{
}

Task::~Task()
{
}

void Task::distance_framesTransformerCallback(const base::Time &ts, const ::base::samples::DistanceImage &distance_frames_sample)
{
    throw std::runtime_error("Transformer callback for distance_frames not implemented");
}
void Task::odometry_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &odometry_samples_sample)
{
    throw std::runtime_error("Transformer callback for odometry_samples not implemented");
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    env = new Environment();
    if( _debug_viz.value() )
    {
	viz.start();
	viz.getWidget()->updateData( env );
    }

    return true;
}

// bool Task::startHook()
// {
//     if (! TaskBase::startHook())
//         return false;
//     return true;
// }
// void Task::updateHook()
// {
//     TaskBase::updateHook();
// }
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

    delete env;
}


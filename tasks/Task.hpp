/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef GRAPH_SLAM_TASK_TASK_HPP
#define GRAPH_SLAM_TASK_TASK_HPP

#include "graph_slam/TaskBase.hpp"
#include <vizkit/EnvireWidget.hpp>
#include <vizkit/QtThreadedWidget.hpp>

namespace envire
{
    class Environment;
}

namespace vizkit
{
    class EnvireVisualization;
}

namespace graph_slam {

    class PoseGraph;

    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:

        virtual void odometry_delta_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &odometry_delta_samples_sample);
        virtual void distance_framesTransformerCallback(const base::Time &ts, const ::base::samples::DistanceImage &distance_frames_sample);
        virtual void stereo_featuresTransformerCallback(const base::Time &ts, const ::stereo::StereoFeatureArray &feature_arrays_sample);
        virtual void texture_imagesTransformerCallback(const base::Time &ts, const ::base::samples::frame::Frame &texture_images_sample);

	QtThreadedWidget<envire::EnvireWidget> viz;

	envire::Environment *env;
	envire::TransformWithUncertainty body2PrevBody, lastBody2Odometry;
	bool firstNode;
	PoseGraph *graph;

	bool lastFeatureArrayValid;
	stereo::StereoFeatureArray lastFeatureArray;

	bool lastTextureImageValid;
	base::samples::frame::Frame textureImage;

    public:
        Task(std::string const& name = "graph_slam::Task");
        Task(std::string const& name, RTT::ExecutionEngine* engine);

	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         *
         *   task_context "TaskName" do
         *     needs_configuration
         *     ...
         *   end
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        // bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        // void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        // void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif


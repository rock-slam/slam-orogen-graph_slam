/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef GRAPH_SLAM_VELODYNESLAM_TASK_HPP
#define GRAPH_SLAM_VELODYNESLAM_TASK_HPP

#include "graph_slam/VelodyneSLAMBase.hpp"

#include <envire/Orocos.hpp>
#include <envire/core/EventHandler.hpp>
#include <boost/shared_ptr.hpp>
#include <graph_slam/extended_sparse_optimizer.hpp>

namespace graph_slam {
    
    class MLSGridEventFilter : public envire::EventFilter
    {
    public:
        MLSGridEventFilter() : EventFilter() {}
        virtual bool filter( envire::Event const& event )
        {
            envire::MultiLevelSurfaceGrid* mls_grid = dynamic_cast<envire::MultiLevelSurfaceGrid*>(event.a.get());
            if((event.type == envire::event::ITEM || event.type == envire::event::FRAMENODE) && mls_grid)
            {
                return true;
            }
            return false;
        }
    };

    /*! \class VelodyneSLAM 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','graph_slam::VelodyneSLAM')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class VelodyneSLAM : public VelodyneSLAMBase
    {
	friend class VelodyneSLAMBase;
    protected:
        boost::shared_ptr<envire::OrocosEmitter> orocos_emitter;
        boost::shared_ptr<MLSGridEventFilter> event_filter;
        base::Time last_envire_update;
        envire::TransformWithUncertainty last_odometry_transformation;
        graph_slam::ExtendedSparseOptimizer optimizer;
        unsigned try_edges_on_update;
        unsigned new_vertecies;
        unsigned edge_count;
        bool map_updated;
        States last_state;
        States new_state;
        VelodyneSlamDebug debug_information;
        base::Time last_new_vertex;

    protected:
        void handleLidarData(const base::Time &ts, const velodyne_lidar::MultilevelLaserScan* lidar_sample, const base::samples::Pointcloud* simulated_pointcloud_sample = NULL);
        virtual void lidar_samplesTransformerCallback(const base::Time &ts, const ::velodyne_lidar::MultilevelLaserScan &lidar_samples_sample);
        virtual void simulated_pointcloudTransformerCallback(const base::Time &ts, const ::base::samples::Pointcloud &simulated_pointcloud_sample);
        virtual bool generateMap();
        void writeOptimizerDebugInformation();

    public:
        /** TaskContext constructor for VelodyneSLAM
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        VelodyneSLAM(std::string const& name = "graph_slam::VelodyneSLAM");

        /** TaskContext constructor for VelodyneSLAM 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        VelodyneSLAM(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of VelodyneSLAM
         */
	~VelodyneSLAM();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

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
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif


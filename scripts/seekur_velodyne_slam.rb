require 'vizkit'
require 'orocos'
require 'orocos/log'
require "transformer/runtime"

include Orocos
Orocos::CORBA.max_message_size = 80000000

if not ARGV[0]
    puts "add a valid logfile folder as parameter"
    exit
end

log = Orocos::Log::Replay.open(ARGV)
log.use_sample_time = true

Orocos.initialize

# find ports
velodyne_ports = log.find_all_output_ports("/velodyne_lidar/MultilevelLaserScan", "laser_scans")
task_states = log.find_all_output_ports("/int32_t", "state")
if log.has_task?("odometry") then
    odometry_port = log.find_all_output_ports("/base/samples/RigidBodyState_m", "odometry_samples")
    Orocos.transformer.load_conf(File.join(File.dirname(__FILE__),"seekur_transforms.rb"))
elsif log.has_task?("seekur_drv") then
    odometry_port = log.find_all_output_ports("/base/samples/RigidBodyState_m", "robot_pose")
    Orocos.transformer.load_conf(File.join(File.dirname(__FILE__),"seekur_sand_track_transforms.rb"))
else
    puts "could not find odometry samples"
    exit
end

# track only needed ports
log.transformer_broadcaster.track(true)
velodyne_ports.each do |port|
    port.tracked = true
end
odometry_port.each do |port|
    port.tracked = true
end
# have to track all states, this seems to be a bug
task_states.each do |port|
    port.tracked = true
end

Orocos.run "graph_slam::VelodyneSLAM" => "velodyne_slam" do

    #Orocos.log_all_ports
    
    velodyne_slam = TaskContext.get 'velodyne_slam'
    #velodyne_slam.envire_path = "#{ENV['AUTOPROJ_PROJECT_BASE']}/slam/orogen/graph_slam/env/"
    velodyne_slam.envire_period = 1.0
    velodyne_slam.grid_size_x = 100
    velodyne_slam.grid_size_y = 100
    velodyne_slam.vertex_distance = 2.0
    velodyne_slam.max_icp_distance = 7.0
    velodyne_slam.grid_size_x = 200
    velodyne_slam.grid_size_y = 200
    velodyne_slam.cell_resolution_x = 0.1
    velodyne_slam.cell_resolution_y = 0.1

    # connect ports with the task
    velodyne_ports.each do |port|
        port.connect_to velodyne_slam.lidar_samples, :type => :buffer, :size => 100
    end
    odometry_port.each do |port|
        port.connect_to velodyne_slam.odometry_samples
        # frame names are not set in the seekur_drv deployment
        if log.has_task?("seekur_drv") then
            port.filter = Proc.new do |frame|
                frame.sourceFrame = 'body'
                frame.targetFrame = 'odometry'
                frame
            end
        end
    end

    Orocos.transformer.setup(velodyne_slam)

    velodyne_slam.configure
    velodyne_slam.start

    ## setup vizkit visualizations
    view3d = Vizkit.vizkit3d_widget
    view3d.show
    envireviz = Vizkit.default_loader.EnvireVisualization
    bodystateviz = Vizkit.default_loader.RigidBodyStateVisualization
    Vizkit.connect_port_to 'velodyne_slam', 'envire_map', :pull => false, :update_frequency => 33 do |sample, name|
        envireviz.updateBinaryEvents(sample)
    end
    Vizkit.connect_port_to 'velodyne_slam', 'pose_samples', :pull => false, :update_frequency => 33 do |sample, name|
        bodystateviz.updateRigidBodyState(sample)
    end
    
    Vizkit.control log
    Vizkit.display velodyne_slam
    begin
        Vizkit.exec
    rescue Interrupt => e
        velodyne_slam.stop
        velodyne_slam.cleanup
    end
end

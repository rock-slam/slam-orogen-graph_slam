require 'vizkit'
require 'orocos'
require 'orocos/log'
require "transformer/runtime"
require File.join(File.dirname(__FILE__),'gui/graph_slam_gui.rb')
require File.join(File.dirname(__FILE__),'gui/gen_map_button.rb')

include Orocos
Orocos::CORBA.max_message_size = 80000000

if not ARGV[0]
    puts "add a valid logfile folder as parameter"
    exit
end

log = Orocos::Log::Replay.open(ARGV)
log.use_sample_time = false
log.track(false)

Orocos.initialize

# find ports
velodyne_ports = log.find_all_output_ports("/velodyne_lidar/MultilevelLaserScan", "laser_scans")
task_states = log.find_all_output_ports("/int32_t", "state")
odometry_port = log.find_all_output_ports("/base/samples/RigidBodyState_m", "odometry_samples")

# track only needed ports
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

log.velodyne_slam.rename('velodyne_slam_log')
log.transformer_broadcaster.rename('transformer_broadcaster_log')

Orocos.run "graph_slam::VelodyneSLAM" => "velodyne_slam" do

    # load transforms
    Orocos.transformer.load_conf(File.join(File.dirname(__FILE__),"artemis_transforms.rb"))
    #Orocos.log_all_ports
    
    velodyne_slam = TaskContext.get 'velodyne_slam'
    #velodyne_slam.environment_debug_path = "#{ENV['AUTOPROJ_PROJECT_BASE']}/slam/orogen/graph_slam/scripts/env/"
    velodyne_slam.enable_debug = true
    velodyne_slam.envire_period = 1.0
    # distance between the laserscans
    velodyne_slam.vertex_distance = 0.5
    velodyne_slam.max_icp_distance = 3.0
    # create a mls map, otherwise it will send out the raw pointclouds
    velodyne_slam.use_mls = true
    velodyne_slam.grid_size_x = 200
    velodyne_slam.grid_size_y = 200
    velodyne_slam.cell_resolution_x = 0.1
    velodyne_slam.cell_resolution_y = 0.1
    velodyne_slam.footprint_radius = 0.0

    # connect ports with the task
    velodyne_ports.each do |port|
        port.connect_to velodyne_slam.lidar_samples, :type => :buffer, :size => 100
    end
    odometry_port.each do |port|
        port.connect_to velodyne_slam.odometry_samples, :type => :buffer, :size => 1000
    end

    load_generate_map_button(velodyne_slam)

    Orocos.transformer.setup(velodyne_slam)

    velodyne_slam.configure
    velodyne_slam.start

    load_graph_slam_gui(velodyne_slam)

    Vizkit.control log
    Vizkit.display velodyne_slam
    begin
        Vizkit.exec
    rescue Interrupt => e
        velodyne_slam.stop
        velodyne_slam.cleanup
    end
end

require 'vizkit'
require 'orocos'
require 'orocos/log'
require "transformer/runtime"

include Orocos
Orocos::CORBA.max_message_size = 80000000

if not ARGV[0]
    puts "add a valid logfile as parameter"
    exit
end

log = Orocos::Log::Replay.open(ARGV)
log.use_sample_time = true
Vizkit.control log

Orocos.initialize
Orocos.transformer.load_conf(File.join(File.dirname(__FILE__),"seekur_transforms.rb"))

Orocos.run "graph_slam::VelodyneSLAM" => "velodyne_slam" do

    #Orocos.log_all_ports
    
    velodyne_slam = TaskContext.get 'velodyne_slam'
    #velodyne_slam.envire_path = "#{ENV['AUTOPROJ_PROJECT_BASE']}/slam/orogen/graph_slam/env/"
    velodyne_slam.envire_period = 1.0
    velodyne_slam.vertex_distance = 0.5
    velodyne_slam.use_mls = false

    log.velodyne.laser_scans.connect_to velodyne_slam.lidar_samples, :type => :buffer, :size => 100

    Orocos.transformer.setup(velodyne_slam)

    velodyne_slam.configure
    velodyne_slam.start

    ## setup vizkit visualizations
    view3d = Vizkit.vizkit3d_widget
    view3d.show
    envireviz = Vizkit.default_loader.EnvireVisualization
    Vizkit.connect_port_to 'velodyne_slam', 'envire_map', :pull => false, :update_frequency => 33 do |sample, name|
        envireviz.updateBinaryEvents(sample)
    end
    
    #Vizkit.display log.velodyne.laser_scans
    Vizkit.display velodyne_slam
    begin
        Vizkit.exec
    rescue Interrupt => e
        velodyne_slam.stop
        velodyne_slam.cleanup
    end
end

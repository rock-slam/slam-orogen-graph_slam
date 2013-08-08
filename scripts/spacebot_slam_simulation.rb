require 'vizkit'
require 'orocos'
require "transformer/runtime"

include Orocos
Orocos::CORBA.max_message_size = 8000000

Orocos.initialize
Orocos.transformer.load_conf(File.join(File.dirname(__FILE__),"simulation_transforms.rb"))

# Content of these language variables may not be german, otherwise '.' and ','
# are mixed reading numbers and a bad_alloc error occurs loading the scenes.
ENV['LANG'] = 'C'
ENV['LC_NUMERIC'] = 'C'

Orocos.run 'spacebot_simulation', 'graph_slam::VelodyneSLAM' => 'velodyne_slam', 
            'transformer::Task' => 'transformer_broadcaster', 
            'spacebot_motioncontroller::Task' => 'controller',
            "valgrind" => false, "wait" => 1000 do

    Orocos.conf.load_dir('./config')

    # SIMULATION
    simulation = TaskContext.get 'mars_simulation'  
    simulation.apply_conf(['default'])
    simulation.configure
    simulation.start
    # Has to be called after configure.
    simulation.loadScene("#{ENV['AUTOPROJ_PROJECT_BASE']}/install/configuration/mars_scenes/spaceBot.scn")
    
    # ACTUATORS
    locomotion_actuators_names = ["rear_left", "rear_left_turn", "middle_left", "middle_left_turn", "front_right", "front_right_turn", 
            "front_left", "front_left_turn", "rear_right", "rear_right_turn", "middle_right", "middle_right_turn"]
    locomotion_actuators_indices = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12] # Disable actuator by using 0, invert by using a negative index. 
    locomotion_actuators = TaskContext.get 'mars_locomotion'
    locomotion_actuators.names = locomotion_actuators_names
    if not locomotion_actuators.dispatch("locomotion_actuators", locomotion_actuators_indices) # Has to be executed before configure
        puts "Locomotion actuators could not be dispatched (not available in the scene file?), exit"
        exit 1
    end
    locomotion_actuators.configure
    locomotion_actuators.start

    # IMU
    imu = TaskContext.get 'mars_imu'
    imu.apply_conf(['default'])
    
    # VELODYNE
    velodyne = TaskContext.get 'mars_velodyne'
    velodyne.apply_conf(['default'])
    velodyne.configure
    velodyne.start   
    velodyne.addCamera('velodyne90',90);
    velodyne.addCamera("velodyne180",180);
    velodyne.addCamera("velodyne270",270);

    # CONTROLLER
    controller = TaskContext::get 'controller'
    controller.ackermann_ratio_p = 1.0
    controller.start
    
    # JOYSTICK
    motion_cmd_writer = controller.motion_command.writer
    motion_cmd = motion_cmd_writer.new_sample
    joystickGui = Vizkit.default_loader.create_plugin('VirtualJoystick')
    joystickGui.show
    joystickGui.connect(SIGNAL('axisChanged(double, double)')) do |x, y|
        motion_cmd.translation = x
        motion_cmd.rotation =  - y.abs() * Math::atan2(y, x.abs()) / 1.0
        motion_cmd_writer.write(motion_cmd)
    end 
    
    # MAPPING
    velodyne_slam = TaskContext.get 'velodyne_slam'
    velodyne_slam.apply_conf(['default'])
    velodyne_slam.transformer_max_latency = 5.0
    velodyne_slam.simulated_pointcloud_period = 2.5
    velodyne_slam.lidar_samples_period = 2.5

    ## setup transformer broadcaster
    broadcaster = Orocos::TaskContext.get "transformer_broadcaster"
    broadcaster.start
    Transformer.broadcaster = broadcaster 

    Orocos.transformer.setup(velodyne_slam, imu, velodyne)

    imu.configure
    imu.start
    velodyne_slam.configure
    velodyne_slam.start

    # CONNECT PORTS
    imu.pose_samples.connect_to(simulation.pose_in)
    controller.actuators_command.connect_to(locomotion_actuators.cmd_locomotion_actuators)
    velodyne.pointcloud.connect_to velodyne_slam.simulated_pointcloud, :type => :buffer, :size => 100
    imu.pose_samples.connect_to velodyne_slam.odometry_samples

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
    
    Vizkit.display velodyne_slam
    begin
        Vizkit.exec
    rescue Interrupt => e
        velodyne_slam.stop
        velodyne_slam.cleanup
    end
end

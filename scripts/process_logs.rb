#! /usr/bin/env ruby

require 'vizkit'
require 'asguard'
require 'optparse'
include Orocos

def usage
    STDERR.puts 
    exit 1
end

out_file = "/tmp/env"

opt_parse = OptionParser.new do |opt|
    opt.banner = "process_logs.rb <log_file_dir>"
    opt.on("-o output_environment", String, "path to output environment") do |name|
	out_file = name
    end
end

args = opt_parse.parse( ARGV )
if args.size < 1
    puts opt_parse
    exit 1
end

log_file = args[0]
# filter out properties.log files
replay = Asguard::Replay.new( log_file )

Orocos.initialize
Orocos.load_typekit "stereo"
Orocos::Process.run 'graph_slam_test', 'valgrind'=>false, "wait" => 1000 do |p|
    graph_slam = p.task('graph_slam')

    graph_slam.environment_debug_path = out_file

    replay.log.odometry.odometry_delta_samples.connect_to( graph_slam.odometry_delta_samples, :type => :buffer, :size => 1000 )
    replay.log.dynamixel.lowerDynamixel2UpperDynamixel.connect_to( graph_slam.dynamic_transformations, :type => :buffer, :size => 1000 )
    replay.log.stereo.distance_frame.connect_to( graph_slam.distance_frames, :type => :buffer, :size => 2 )
    replay.log.stereo.stereo_features.connect_to( graph_slam.stereo_features, :type => :buffer, :size => 2 )

    tf = Asguard::Transform.new [:dynamixel]
    tf.setup_filters replay

    replay.log.align( :use_sample_time )
    graph_slam.debug_viz = true

    graph_slam.configure
    graph_slam.start
    tf.configure_task graph_slam

    Vizkit.control replay.log
    Vizkit.exec
end

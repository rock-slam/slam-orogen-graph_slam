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
Orocos::Process.spawn 'graph_slam_test', 'valgrind'=>false, "wait" => 1000 do |p|
    graph_slam = p.task('graph_slam')

    graph_slam.environment_debug_path = out_file

    replay.log.odometry.odometry_samples.connect_to( graph_slam.dynamic_transformations, :type => :buffer, :size => 1000 )
    replay.log.dynamixel.lowerDynamixel2UpperDynamixel.connect_to( graph_slam.dynamic_transformations, :type => :buffer, :size => 1000 )
    if replay.log.has_task? :stereo and replay.log.stereo.has_port? :distance_frame
        replay.log.stereo.distance_frame.connect_to( graph_slam.distance_frames, :type => :buffer, :size => 2 )
        puts "INFO: Using distance images."
    end

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

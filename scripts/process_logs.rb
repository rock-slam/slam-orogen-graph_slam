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

    replay.log.odometry.bodystate_samples.connect_to( graph_slam.bodystate_samples, :type => :buffer, :size => 1000 )
    replay.log.odometry.odometry_samples.connect_to( graph_slam.dynamic_transformations, :type => :buffer, :size => 1000 )
    replay.log.dynamixel.lowerDynamixel2UpperDynamixel.connect_to( graph_slam.dynamic_transformations, :type => :buffer, :size => 1000 )
    if @log_replay.has_task? :dense_stereo and @log_replay.dense_stereo.has_port? :distance_frame
        @log_replay.dense_stereo.distance_frame.connect_to( @eslam.distance_frames, :type => :buffer, :size => 2 )
        @has_distance_images = true
        puts "INFO: Using distance images."
    end

    tf = Asguard::Transform.new [:dynamixel]
    tf.setup_filters replay

    graph_slam.configure
    graph_slam.start
    tf.configure_task graph_slam

    replay.log.run
end

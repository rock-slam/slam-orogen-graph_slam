def load_generate_map_button(velodyne_slam_task)
    widget = Vizkit::load File.join(File.dirname(__FILE__),'gen_map.ui')

    widget.generate_map_button.connect(SIGNAL('released()')) do
        widget.generate_map_button.enabled = false
        velodyne_slam_task.generateMap
        widget.generate_map_button.enabled = true
    end

    widget.show
end

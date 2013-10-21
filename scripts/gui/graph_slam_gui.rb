def load_graph_slam_gui(velodyne_slam_task)
    widget = Vizkit::load File.join(File.dirname(__FILE__),'graph_slam_gui.ui')
    semaphore = Mutex.new
    graphics_scene = Qt::GraphicsScene.new
    widget.graph_viz_img_view.scene = graphics_scene
    Vizkit.vizkit3d_widget = widget.vizkit_widget

    widget.update_graph_viz.connect(SIGNAL('released()')) do
        dot_file = File.new("/tmp/graph_viz.dot", "w")
        semaphore.synchronize { dot_file.write(widget.graph_viz_dot.plainText) }
        dot_file.close
        neato_ok = system( "neato -Tpng /tmp/graph_viz.dot -o /tmp/graph_viz.png" )
        widget.graph_viz_img_view.scene.clear
        if neato_ok then
            widget.graph_viz_img_view.scene.addItem Qt::GraphicsPixmapItem.new(Qt::Pixmap.new("/tmp/graph_viz.png"))
        end
    end

    if velodyne_slam_task.is_a?(Orocos::Async::TaskContextProxy) then
        velodyne_slam_task.port("debug_information").on_data do |data|
            vertical_scrollbar_value = widget.graph_viz_dot.verticalScrollBar.value
            semaphore.synchronize { widget.graph_viz_dot.plainText = data.graphviz }
            widget.graph_viz_dot.verticalScrollBar.setValue(vertical_scrollbar_value)
        end

        velodyne_slam_task.port("envire_map").once_on_reachable do
            Vizkit.display velodyne_slam_task.port("envire_map")
        end

        velodyne_slam_task.port("pose_samples").once_on_reachable do
            Vizkit.display velodyne_slam_task.port("pose_samples"), :widget => Vizkit.default_loader.RigidBodyStateVisualization
        end

    elsif velodyne_slam_task.is_a?(Orocos::TaskContext)
        velodyne_slam_task.debug_information.connect_to do |data,_|
            vertical_scrollbar_value = widget.graph_viz_dot.verticalScrollBar.value
            semaphore.synchronize { widget.graph_viz_dot.plainText = data.graphviz }
            widget.graph_viz_dot.verticalScrollBar.setValue(vertical_scrollbar_value)
        end

        Vizkit.display velodyne_slam_task.envire_map
        Vizkit.display velodyne_slam_task.pose_samples, :widget => Vizkit.default_loader.RigidBodyStateVisualization
    else
        puts "error velodyne_slam_task is no valid TaskContext"
        exit
    end

    widget.show
    widget
end

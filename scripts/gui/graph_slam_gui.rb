def load_graph_slam_gui(velodyne_slam_task)
    widget = Vizkit::load File.join(File.dirname(__FILE__),'graph_slam_gui.ui')
    Vizkit.vizkit3d_widget = widget.vizkit_widget

    debug_reader = velodyne_slam_task.debug_information.reader

    semaphore = Mutex.new
    graphviz_timer = Qt::Timer.new
    graphics_scene = Qt::GraphicsScene.new
    widget.graph_viz_img_view.scene = graphics_scene
    graphviz_timer.connect(SIGNAL('timeout()')) do
        sample = debug_reader.read_new
        if not sample.nil? then
            semaphore.synchronize { widget.graph_viz_dot.plainText = sample.graphviz }
        end
    end
    graphviz_timer.start(5000)

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

    Vizkit.display velodyne_slam_task.envire_map
    Vizkit.display velodyne_slam_task.pose_samples, :widget => Vizkit.default_loader.RigidBodyStateVisualization

    widget.show
    widget
end

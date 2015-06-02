static_transform(Eigen::Vector3.new(0.0, 0.0, 0.641305),
    Eigen::Quaternion.from_euler(Eigen::Vector3.new(0,0,0),2,1,0),
    "laser" => "body")

dynamic_transform "odometry.odometry_samples", "body" => "odometry"

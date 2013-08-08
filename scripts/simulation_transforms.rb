static_transform(Eigen::Vector3.new(-0.03838000074028969,0.0012229999992996454,0.5270000100135803),
                 Eigen::Quaternion.from_euler(Eigen::Vector3.new(0,0,0),2,1,0),"laser" => "body")

static_transform(Eigen::Vector3.new(0.0,0.0,0.0),
                 Eigen::Quaternion.from_euler(Eigen::Vector3.new(0,0,0),2,1,0),"imu" => "body")

dynamic_transform "mars_imu.pose_samples", "imu" => "world"

static_transform(Eigen::Vector3.new(0.0,0.0,0.0),
                 Eigen::Quaternion.from_euler(Eigen::Vector3.new(0,0,0),2,1,0),"odometry" => "world")

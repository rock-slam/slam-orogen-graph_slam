static_transform(Eigen::Vector3.new(-0.503,0.0,1.364),
                 Eigen::Quaternion.from_euler(Eigen::Vector3.new(-0.5*Math::PI,0,0),2,1,0),"laser" => "body")

static_transform(Eigen::Vector3.new(0.0,0.0,0.0),
		 Eigen::Quaternion.from_euler(Eigen::Vector3.new(0,0,0),2,1,0),"map" => "world")

static_transform(Eigen::Vector3.new(0.0,0.0,0.0),
		 Eigen::Quaternion.from_euler(Eigen::Vector3.new(0,0,0),2,1,0),"odometry" => "world")

dynamic_transform "seekur_drv.robot_pose", "body" => "odometry"

#include <ros/ros.h>
#include <ros/transform_broadcaster.h>

int main (int argc, char** argv){
	ros::init(argc, argv, "robot_tf_publisher");
	ros::NodeHandle n;
	
	double tilt_angle, camera_height;
	
	n.param("tilt_angle", tilt_angle, 0.3);
	n.param("camera_height", camera_height, 0.55);
	
	ros::Rate rate_loop(50);
	
	tf::TransformBroadcaster broadcaster;
	
	while(n.ok()){
		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(tilt_angle,0 ,0), tf::Vector3(0, 0,camera_height)),
				ros::Time::now(), "base_robot", "camera_link"
			)
		);
		
		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0,0,0), tf::Vector3(-0.2, 0, 0.1)),
				ros::Time::now(), "base_robot", "base_link"
			)
		);
		
		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0,0,0), tf::Vector3(0,0,0.3)),
				ros::Time::now(), "base_robot", "laser_link"
			)
		);
		
		rate_loop.sleep();
	}
}

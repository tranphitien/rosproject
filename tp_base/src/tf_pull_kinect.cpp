#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

//#define KINECT_TILT_ANGLE 0.26125	//a fixed 16-deg rotation
//#define KINECT_DIST2GROUND 0.4		//a 0.4m distance from ground

int main(int argc, char** argv){
	ros::init(argc, argv, "robot_tf_publisher");
	ros::NodeHandle n;

	double tilt_angle;
	double camera_height;

	n.param("tilt_angle", tilt_angle, 0.2705);
	n.param("camera_height", camera_height, 0.4);

	ros::Rate r(50);

	tf::TransformBroadcaster broadcaster;

	while(n.ok())
	{
		broadcaster.sendTransform
		(
				tf::StampedTransform
				(
						tf::Transform(tf::Quaternion(tilt_angle, 0, 0), tf::Vector3(0.0, 0.0, camera_height)),
						ros::Time::now(),
						"ngatalie_base",
						"camera_link"
				)
		);

		broadcaster.sendTransform
		(
				tf::StampedTransform
				(
						tf::Transform(tf::Quaternion(0, 0, 0), tf::Vector3(-0.2, 0.0, 0.1)),
						ros::Time::now(),
						"ngatalie_base",
						"rect_base"
				)
		);

		broadcaster.sendTransform
		(
				tf::StampedTransform
				(
						tf::Transform(tf::Quaternion(0, 0, 0), tf::Vector3(0.0, 0.0, 0.30)),
						ros::Time::now(),
						"ngatalie_base",
						"laser_link"
				)
		);
		r.sleep();
	}
}

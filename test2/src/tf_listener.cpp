#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

//we'll create a function that, given a TransfofmListener, takes a point in the base_laser frame and transforms it to 
//the base_link frame. this finction will serve as a callback for the ros::Timer created in the main() of our program and
//will fire every second
void transformPoint(const tf::TransformListener& listener){

	//Here, we'll create our point as a geometry_msgs::PointStamped. the Stamped on the end of the message name just
	//means that it includes a header, allowing is to associate both a timestamp and a frame_id with the message
	//we'll set the stamp field of laser_point message to be ros::Time() which is a special time value the allows us to ask the 
	//TransformListener for the latest available transform. As for the frame_id field of the header, we'll set that to be 
	//base_laser because we're creating a point in te base_laser frame. Finally, we'll set some date for the point
	//we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
	geometry_msgs::PointStamped laser_point;
	laser_point.header.frame_id = "laser_link";
	//we'll just use the most recent transform available for our simple example
	laser_point.header.stamp = ros::Time();
	
	//just an arbitrary point in space
	//laser_point.point.x = 1.0;
	//laser_point.point.y = 2.0;
	//laser_point.point.z = 0.0;
	
	try{
	geometry_msgs::PointStamped base_point;
	listener.transformPoint("base_link", laser_point, base_point);
	
	ROS_INFO("laser_link: (%.2f, %.2f, %.2f) ----> base_link: (%.3f, %.3f, %.3f) at time %.3f",
				laser_point.point.x, laser_point.point.y, laser_point.point.z,
				base_point.point.x, base_point.point.y, base_point.point.z,
				base_point.header.stamp.toSec());
	}
	//if for some reason the base_laser->base_link transform is unavailable(perhaps the tf_broadcaster is not running)
	//the TransformListener may throw an exception when we call transformPoint().
	//to make sure we handle this gracefully, we'll catch the exception and print out an error for the user
	catch(tf::TransformException& ex){
		ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
	}
}


int main(int argc, char** argv){
	ros::init(argc, argv, "robot_tf_listener");
	ros::NodeHandle n;
	
	tf::TransformListener listener(ros::Duration(10));
	
	//we'll transform a point once every second
	ros::Timer timer = n.createTimer(ros::Duration(1.0),
												boost::bind(&transformPoint, boost::ref(listener)));
	
	ros::spin();
}

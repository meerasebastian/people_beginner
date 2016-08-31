#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <people_msgs/PositionMeasurementArray.h>





void peoplePoseCallback(const people_msgs::PositionMeasurementArray::ConstPtr& msg){
  ROS_INFO("I can hear!");
  
  
  for (int i = 0; i < msg->people.size(); i++)
  {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->people[i].pos.x, msg->people[i].pos.y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0); //I think this should be zero
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_combined", msg->people[i].name));
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "people_tf_broadcaster");
  
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("people_tracker_measurements", 10, &peoplePoseCallback);

  ros::spin();
  return 0;
};
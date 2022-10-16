#include<ros/ros.h>
#include<geometry_msgs/Vector3.h>
#include<geometry_msgs/Pose2D.h>
#include<math.h>
geometry_msgs::Vector3 velocity;
geometry_msgs::Pose2D pose;

void myCallback(const geometry_msgs::Vector3& msg){
    //checks for messages on topic "cmd_vel"
    ROS_INFO("received velocity value: %f", message_holder.data);
    velocity.linear = msg.linear;
    velocity.angular = msg.angular;
}

int main(int argc, char **argv){
   ros::init(argc, argv, "simulator_pose"); //name the node as simulator_pose
   ros::NodeHandle nh; //node handle
   //create Subscriber object that subscribes to the topic "cmd_vel"
   ros::Subscriber my_subscriber_object = nh.subscribe("cmd_vel", 1, myCallback);
   //publish resulting pose
   ros::Publisher my_publisher_object = nh.advertise<geometry_msgs::Pose2D>("pose", 1);
   //variables for calcuation
   double l = 1.0;
   double dt = 0.01;
   double sample_rate = 1.0 / dt;
   ros::Rate naptime(sample_rate);
   velocity.linear.x = 0.0; //initialize linear to 0, will get updated by callback
   velocity.linear.y = 0.0;
   velocity.linear.z = 0.0;
   velocity.angular.x = 0.0; //initialize angular to 0, will get updated by callback
   velocity.angular.y = 0.0;
   velocity.angular.z = 0.0;
   pose.x = 0.0; //initialize pose to (0,0, pi/2)
   pose.y = 0.0;
   pose.theta = M_PI/2; 
   while(ros::ok()){
      pose.x = pose.x + velocity.linear.x;//equation 
      pose.y = pose.y + velocity.linear.y;
      pose.theta = atan2(pose.y,pose.x);
      my_publisher_object.publish(pose); //publish the system state
      ROS_INFO_STREAM("current pose:"<<"x = "<<pose.x<<"y = "<<pose.y<<"theta = "<<pose.theta);
      ros::spinOnce(); //allow data update from callback
      naptime.sleep(); //wait for remainder of specifieid period
   }
   return 0; //should never get here unless roscore dies
}

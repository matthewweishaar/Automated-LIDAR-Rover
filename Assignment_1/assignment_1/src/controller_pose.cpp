#include<ros/ros.h>
#include<geometry_msgs/Pose2D.h>
#include<geometry_msgs/Vector3.h>
#include<math.h>
geometry_msgs::Pose2D curr_pose;
geometry_msgs::Pose2D final_pose;
geometry_msgs::Vector3 velocity;

void myCallbackFinalPose(const geometry_msgs::Pose2D& msg){
   //check for data on final/goal pose
   ROS_INFO("received goal pose value is: (%f,%f,%f)", msg.x, msg.y, msg.theta);
   final_pose.x = msg.x; //place callback values into global variable
   final_pose.y = msg.y;
   final_pose.theta = msg.theta;
}

void myCallbackCurrPose(const geometry_msgs::Pose2D& msg){
   //check for data on pose
   ROS_INFO("received current pose value is: (%f,%f,%f)", msg.x, msg.y, msg.theta);
   curr_pose.x = msg.x; //place callback values into global variable
   curr_pose.y = msg.y;
   curr_pose.theta = msg.theta;
} 

int main(int argc, char **argv){
   ros::init(argc, argv, "controller_pose"); //name node controller_pose
   ros::NodeHandle nh; //node handle
   //2 subscrivers that will take the current pose (from simulator) and final pose (input)
   ros::Subscriber my_subscriber_object1 = nh.subscribe("pose", 1, myCallbackCurrPose);
   ros::Subscriber my_subscriber_object2 = nh.subscrive("final_pose", 1, myCallbackFinalPose);
   //publisher node for linear and angular
   ros::Publisher my_publisher_object = nh.advertise<geometry_msgs::Vector3>("cmd_vel", 1);
   
   //initializing variables to 0
   double inc_x = 0.0;
   double inc_y = 0.0;
   double angle_to_goal = 0.0;
   final_pose.x = 0.0;
   final_pose.y = 0.0;
   final_pose.theta = 0.0;
   curr_pose.x = 0.0;
   curr_pose.y = 0.0;
   curr_pose.theta = 0.0;
   velocity.linear.x = 0.0;
   velocity.linear.y = 0.0;
   velocity.linear.z = 0.0;
   velocity.angular.x = 0.0;
   velocity.angular.y = 0.0;
   velocity.angular.z = 0.0;
   
   double dt_controller = 0.1; //10Hz controller sample rate
   double sample_rate / dt_controller = 1.0;
   ros::Rate naptime(sample_rate); //regulates loop rate
   
   //enter main loop to control the angle in order to reach the goal pose from the current pose
   while(ros::ok()){
      //calculate how much further we need to go to get to the goal pose
      inc_x = final_pose.x - curr_pose.x;
      inc_y = final_pose.y - curr_pose.y;
      angle_to_goal = atan2(inc_y, inc_x);
      
      //adjust the angle and speed to properly reach the goal
      if (abs(angle_to_goal - curr_pose.theta) > 0.1){
         velocity.linear.x = 0.0;
         velocity.angular.z = 0.3;
      }
      else{
         velocity.linear.x = 0.5;
         velocity.angular.z = 0.0;
      }
      
      my_publisher.object.publish(velocity); //publish the resulting velocity
      //controller printout
      ROS_INFO("velocity command:\n linear: (%f,%f,%f)\n angular: (%f,%f,%f)\n\n",
      velocity.linear.x, velocity.linear.y, velocity.linear.z, 
      velocity.angular.x, velocity.angular.y, velocity.angular.z);
      
      ros::spinOnce(); //allow data update from callback
      naptime.sleep(); //wait for the remainder of specified period.
   }
   return 0; //should never get here unless roscore dies
}






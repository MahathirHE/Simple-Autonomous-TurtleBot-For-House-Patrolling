#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>

/** define Item and Job ID **/

using namespace std;

std::string path_to_sounds;

/** function declarations **/
bool moveToGoal(double goalXcoor, double goalYcoor);
void objectCallback();

/** declare the coordinates of houses**/
double xKitchen = 1.71876;
double yKitchen = -1.04489;

double xLivingRoom = 1.67841;
double yLivingRoom = -0.15888;

double xToilet = 1.69219;
double yToilet = -2.13292;

bool goalReached = false;

int id=0;
int counter = 0;

void objectCallback(const std_msgs::Float32MultiArrayPtr &object)
{

        if (counter == 0){
          ROS_INFO("GO TO KITCHEN");
          goalReached = moveToGoal(xKitchen, yKitchen);

          ros::Duration(2.0).sleep();
        }

        if (counter == 1){
          ROS_INFO("GO TO LIVING ROOM");
          goalReached = moveToGoal(xLivingRoom, yLivingRoom);

          ros::Duration(2.0).sleep();
        }

        if (counter == 2){
          ROS_INFO("GO TO TOILET");
          goalReached = moveToGoal(xToilet, yToilet);

          ros::Duration(2.0).sleep();
        }

         counter = counter + 1;

}

 int main(int argc, char** argv){
   ros::init(argc, argv, "map_navigation_node");
   ros::NodeHandle n;
   ros::Rate loop_rate(50);
   ros::Subscriber sub = n.subscribe("/objects", 1, objectCallback);

   while (ros::ok())
   {
      //ROS_INFO("Scan your JOB ID in front of robot camera");
      ros::spinOnce();
      loop_rate.sleep();
   }

   return 0;
}
bool moveToGoal(double goalXcoor, double goalYcoor){

   //define a client for to send goal requests to the move_base server through a SimpleActionClient
   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

   //wait for the action server to come up
   while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
   }

   move_base_msgs::MoveBaseGoal goal;

   //set up the frame parameters
   goal.target_pose.header.frame_id = "map";
   goal.target_pose.header.stamp = ros::Time::now();

   /* moving towards the goal*/

   goal.target_pose.pose.position.x =  goalXcoor;
   goal.target_pose.pose.position.y =  goalYcoor;
   goal.target_pose.pose.position.z =  0.0;
   goal.target_pose.pose.orientation.x = 0.0;
   goal.target_pose.pose.orientation.y = 0.0;
   goal.target_pose.pose.orientation.z = 0.0;
   goal.target_pose.pose.orientation.w = 1.0;

   ROS_INFO("Navigating ...");
   ac.sendGoal(goal);

   ac.waitForResult();

   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Arrived!");

      return true;
   }
   else{
      ROS_INFO("Not Arrived!");
      return false;
   }



}

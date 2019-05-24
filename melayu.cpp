#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>

#define BAPA 1
#define KUCING 2
#define EMAK 4

using namespace std;

std::string path_to_sounds;

/** function declarations **/
void objectCallback();

/** declare the coordinates of interest **/

int id=0;

void objectCallback(const std_msgs::Float32MultiArrayPtr &object)
{
   if (object->data.size() > 0)
   {
      id = object->data[0];

      switch (id)
      {
      case BAPA:
         ROS_INFO("Bapa is here!!!");
         break;
      case EMAK:
         ROS_INFO("Emak is here!!!");
         break;
      case KUCING:
         ROS_INFO("Kucing is here!!!");
         break;
      default: // other object
         ROS_INFO("The recognized object is not define in the coding.");
      }
   }
   else
   {
      // No object detected
      ROS_INFO("I have detected nothing :(");
   }
}

 int main(int argc, char** argv){
   ros::init(argc, argv, "patrol_image_node");
   ros::NodeHandle n;
   ros::Rate loop_rate(50);
   ros::Subscriber sub = n.subscribe("/objects", 1, objectCallback);

   while (ros::ok())
   {
      //ROS_INFO("Listening to recognized object id");
      ros::spinOnce();
      loop_rate.sleep();
   }

   return 0;
}

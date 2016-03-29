#include <ros/ros.h>

#include <std_msgs/String.h>

#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>

#include <sensor_msgs/LaserScan.h>
#include <people_beginner/peopleDetails.h>
#include <visualization_msgs/Marker.h>

#include<people_beginner/peopleDetails.h>
#include<people_beginner/peopleHallwayFeatures.h>
#include<hallway/hallwayMsg.h>

#include<string.h>
#include<math.h>


using namespace std;

struct distanceFromPR2{
  string frame_id;
  string  personId;
  float distanceFromPR2;
  float distance_travelled;
  double t0, t1, speed, xdistance, ydistance;
}peopleData[100]; 

string peoplename[100];
int noOfPeople = 0;

ros::Publisher people_features, hallway_pub;
people_beginner::peopleDetails personDetails;

void peoplePositionCallback(const people_msgs::PositionMeasurementArray::ConstPtr& msg){
  
  people_msgs::PositionMeasurementArray peopleArray;
  people_msgs::PositionMeasurement peoplePos;   
  int newPersonFound = 0;
  float currentDistance;
  std::vector<people_msgs::PositionMeasurement> people;
  
  for( int i = 0; i < msg->people.size(); i++ ){
    for( int j = 0; j < noOfPeople; j++){
      newPersonFound = 0;
      
      //Tracking the person
      if( strcmp(msg->people[i].name.c_str() , peopleData[j].personId.c_str()) == 0 ) {
        
        if(peopleData[j].xdistance!=msg->people[i].pos.x && peopleData[j].ydistance!=msg->people[i].pos.y){
          /*ROS_INFO("\n\tPerson Details ");
          ROS_INFO("Person Id : %s", peopleData[j].personId.c_str());
          ROS_INFO("Previous Distance : %f", sqrt(pow(peopleData[j].xdistance,2) + pow(peopleData[j].ydistance,2)));*/
          
          peopleData[j].xdistance = 30.48 * msg->people[i].pos.x; //Converting feet to cm
          peopleData[j].ydistance = 30.48 * msg->people[i].pos.y; //Converting feet to cm
         // ROS_INFO("X distance : %f ", peopleData[j].xdistance);
          //ROS_INFO("Y distance : %f ", peopleData[j].ydistance);
          
          
          currentDistance = sqrt(pow(peopleData[j].xdistance,2)+pow(peopleData[j].ydistance,2));
          peopleData[j].distance_travelled =  abs(currentDistance - peopleData[j].distanceFromPR2);
          peopleData[j].distanceFromPR2 = currentDistance;
          peopleData[j].t0 =  peopleData[j].t1; 
          peopleData[j].t1 = ros::Time::now().toSec();
          peopleData[j].speed =  peopleData[j].distance_travelled / ( peopleData[j].t1 -  peopleData[j].t0);
          /*ROS_INFO("Time T0 : %f", peopleData[j].t0);
          ROS_INFO("Time T1 : %f", peopleData[j].t1);
          ROS_INFO("Speed : %f", peopleData[j].speed);
          ROS_INFO("Current Distance from PR2 : %f", peopleData[j].distanceFromPR2);
          ROS_INFO("Distance travelled : %f", peopleData[j].distance_travelled);*/
          
          //Publish the details
          personDetails.frame_id = "base_link";
          personDetails.personId = peopleData[j].personId; 
          personDetails.distanceFromPR2 = peopleData[j].distanceFromPR2;
          personDetails.t0 = peopleData[j].t0; 
          personDetails.t1 = peopleData[j].t1;
          personDetails.xdistance = peopleData[j].xdistance;
          personDetails.ydistance = peopleData[j].ydistance;
          personDetails.distance_travelled = peopleData[j].distance_travelled; 
          personDetails.speed = peopleData[j].speed;
          people_features.publish(personDetails);  
          
          newPersonFound = 1;
          break;  
        }
      }
    }
    
    //Push the details of new person
    if(newPersonFound == 0){
      peopleData[noOfPeople].frame_id = "base_link";
      peopleData[noOfPeople].personId = msg->people[i].name.c_str();
      peopleData[noOfPeople].t0 = ros::Time::now().toSec();
      peopleData[noOfPeople].t1 = ros::Time::now().toSec();
      peopleData[noOfPeople].xdistance = 30.48 * msg->people[i].pos.x; //Converting feet to cm
      peopleData[noOfPeople].ydistance = 30.48 * msg->people[i].pos.y; //Converting feet to cm
      peopleData[noOfPeople].distanceFromPR2 = sqrt(pow(peopleData[noOfPeople].xdistance,2) + pow(peopleData[noOfPeople].ydistance,2)); //2-D distance
      peopleData[noOfPeople].distance_travelled = 0.0;
      peopleData[noOfPeople].speed = 0.0;
      
      //Publish the details
      personDetails.frame_id = "base_link";
      personDetails.personId = peopleData[noOfPeople].personId; 
      personDetails.distanceFromPR2 = peopleData[noOfPeople].distanceFromPR2;
      personDetails.t0 = peopleData[noOfPeople].t0; 
      personDetails.t1 = peopleData[noOfPeople].t1;
      personDetails.xdistance = peopleData[noOfPeople].xdistance;
      personDetails.ydistance = peopleData[noOfPeople].ydistance;
      personDetails.distance_travelled = peopleData[noOfPeople].distance_travelled; 
      personDetails.speed = peopleData[noOfPeople].speed;
      people_features.publish(personDetails);
      
      /*ROS_INFO("New Person to the structure : %s", peopleData[noOfPeople].personId.c_str());
      ROS_INFO("\n\tPerson Details ");
      ROS_INFO("Person Id : %s added", peopleData[noOfPeople].personId.c_str());
      ROS_INFO("Position : %f    %f", peopleData[noOfPeople].xdistance,peopleData[noOfPeople].ydistance);
      ROS_INFO("Distance from pr2: %f", peopleData[noOfPeople].distanceFromPR2);
      ROS_INFO("Time T0 , T1  : %f , %f", peopleData[noOfPeople].t0, peopleData[noOfPeople].t0);*/
       
      noOfPeople = noOfPeople + 1;
    }
    
  }
  
}

void hallwayDetectionCallback(const hallway::hallwayMsg::ConstPtr& msg){
 people_beginner::peopleHallwayFeatures hallwayRelatedFeatures;
 float m,c, width;
 m = msg->slope_hallwayL; //Slope of Left Hallway
 c = msg->intercept_hallwayL; //Intercept of Left Hallway
 width = msg->width_hallway; //Width of the Hallway
 /*ROS_INFO("\n\tHallway Data");
 ROS_INFO("Slope: %f",m);
 ROS_INFO("Intercept: %f",c);
 ROS_INFO("Width: %f",width);*/
  for( int index = 0; index < noOfPeople; index++){
    
    hallwayRelatedFeatures.frame_id = "base_link";
    hallwayRelatedFeatures.personId = peopleData[index].personId;  
    hallwayRelatedFeatures.distancefromHallwayL = ( abs( m*peopleData[index].xdistance - peopleData[index].ydistance + c ) ) / ( sqrt( pow(m,2) + 1 ) );
    hallwayRelatedFeatures.distancefromHallwayR = msg->width_hallway - hallwayRelatedFeatures.distancefromHallwayL;
    ROS_INFO("\n\tHallway Features");
    ROS_INFO("Person id : %s",hallwayRelatedFeatures.personId.c_str());
    ROS_INFO("Distance from Left Hallway : %f",hallwayRelatedFeatures.distancefromHallwayL);
    ROS_INFO("Distance from Right Hallway : %f",hallwayRelatedFeatures.distancefromHallwayR);
    hallway_pub.publish(hallwayRelatedFeatures);
  }
}

int main( int argc, char* argv[] ){
  
  // Initialize the ROS system and specify the node name.
  ros::init(argc,argv,"legdata") ;
  
  // Establish this program as a ROS node.
  ros::NodeHandle nh ;  
  
  ros::Subscriber people_sub = nh.subscribe("people_tracker_measurements", 1000, peoplePositionCallback);
  ros::Subscriber hallway_sub = nh.subscribe("hallway_data", 1000, hallwayDetectionCallback);
  
  people_features = nh.advertise<people_beginner::peopleDetails>("peopleFeatures",1000);
  hallway_pub = nh.advertise<people_beginner::peopleHallwayFeatures>("peopleHallwayFeatures",1000);
  
  ros::spin();
  
  return 0;
}

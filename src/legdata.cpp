#include <ros/ros.h>

#include <std_msgs/String.h>

#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <people_beginner/peopleDetails.h>
#include <visualization_msgs/Marker.h>

#include<people_beginner/peopleDetails.h>
#include<people_beginner/peopleHallwayFeatures.h>
#include<hallway/hallwayMsg.h>

#include<string.h>
#include<math.h>
#include<fstream>
#include<ctime>


using namespace std;

struct distanceFromPR2{
  string frame_id;
  string  personId;
  float distanceFromPR2;
  float distance_travelled;
  double t0, t1, speed, xdistance, ydistance;
}peopleData[100]; 

string peoplename[100];
int noOfPeople = 0, flag=0,peopleFlag = 0, firstPersonNo = 0;
float robotPositionX = 0, robotPositionY = 0;
double currentTime, startTime;
ofstream outfile ("Dataset1.txt");
ofstream testfile ("Testdataset1.txt");

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
          peopleData[j].xdistance = 100 * msg->people[i].pos.x; //Converting m to cm
          peopleData[j].ydistance = 100 * msg->people[i].pos.y; //Converting m to cm
          
          currentDistance = sqrt(pow((peopleData[j].xdistance - robotPositionX),2)+pow((peopleData[j].ydistance - robotPositionY),2));
          peopleData[j].distance_travelled =  abs(currentDistance - peopleData[j].distanceFromPR2);
          peopleData[j].distanceFromPR2 = currentDistance;
          peopleData[j].t0 =  peopleData[j].t1; 
          peopleData[j].t1 = ros::Time::now().toSec();
          peopleData[j].speed =  peopleData[j].distance_travelled / ( peopleData[j].t1 -  peopleData[j].t0);
          
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
      //Extract Person Id(Number) from Person Name(String)
    
      peopleData[noOfPeople].personId = msg->people[i].name.c_str();
      peopleData[noOfPeople].t0 = ros::Time::now().toSec();
      peopleData[noOfPeople].t1 = ros::Time::now().toSec();
      peopleData[noOfPeople].xdistance = 100 * msg->people[i].pos.x; //Converting m to cm
      peopleData[noOfPeople].ydistance = 100 * msg->people[i].pos.y; //Converting m to cm
      peopleData[noOfPeople].distanceFromPR2 = sqrt(pow((peopleData[noOfPeople].xdistance - robotPositionX),2) + pow((peopleData[noOfPeople].ydistance - robotPositionY),2)); //2-D distance
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
       
      noOfPeople = noOfPeople + 1;
    }
    
  }
  
}

void hallwayDetectionCallback(const hallway::hallwayMsg::ConstPtr& msg){
  double timeStamp;
  string p1ID, p2ID;
  int i;
  float p1HallwayL, p1HallwayR, p2HallwayL, p2HallwayR, p1p2Distance; 
  people_beginner::peopleHallwayFeatures hallwayRelatedFeatures;
  float m,c, width;
  m = msg->slope_hallwayL; //Slope of Left Hallway
  c = msg->intercept_hallwayL; //Intercept of Left Hallway
  width = msg->width_hallway; //Width of the Hallway
  for( int index = 0; index < 1; index++){
    hallwayRelatedFeatures.frame_id = "base_link";
    p1ID = peopleData[index].personId.c_str();  
    p1HallwayR = ( abs( m*peopleData[index].xdistance - peopleData[index].ydistance + c ) ) / ( sqrt( pow(m,2) + 1 ) );
    p1HallwayL = msg->width_hallway - p1HallwayR;
    
    hallwayRelatedFeatures.personId = p1ID;
    hallwayRelatedFeatures.distancefromHallwayR = p1HallwayR;
    hallwayRelatedFeatures.distancefromHallwayL = p1HallwayL;
    hallway_pub.publish(hallwayRelatedFeatures);
    
    for(int innerIndex = 0; innerIndex < noOfPeople; innerIndex++){
      if(innerIndex != index){
        if(flag==0){
          startTime = ros::Time::now().toSec();
          flag = 1;
        }
        ROS_INFO("Value of i : %d", i);
        hallwayRelatedFeatures.frame_id = "base_link";
        p2ID = peopleData[innerIndex].personId.c_str();  
        p2HallwayR = ( abs( m*peopleData[innerIndex].xdistance - peopleData[innerIndex].ydistance + c ) ) / ( sqrt( pow(m,2) + 1 ) );
        p2HallwayL = msg->width_hallway - p2HallwayR;
        
        p1p2Distance = sqrt(pow((peopleData[index].xdistance - peopleData[innerIndex].xdistance),2)+pow((peopleData[index].ydistance - peopleData[innerIndex].ydistance),2));
        currentTime = ros::Time::now().toSec();
        timeStamp = currentTime - startTime;
        
        ROS_INFO("\n\tHallway Features");
        ROS_INFO("Person id : %s",hallwayRelatedFeatures.personId.c_str());
        ROS_INFO("Slope: %f",m);
        ROS_INFO("Intercept: %f",c);
        ROS_INFO("Width: %f",width);
        ROS_INFO("P1 Distance from Left Hallway : %f",p1HallwayL);
        ROS_INFO("P1 Distance from Right Hallway : %f",p1HallwayR);
        ROS_INFO("P2 Distance from Left Hallway : %f",p2HallwayL);
        ROS_INFO("P2 Distance from Right Hallway : %f",p2HallwayR);
        ROS_INFO("P1 position (x, y) : %f , %f", peopleData[index].xdistance, peopleData[index].ydistance);
        ROS_INFO("P2 position (x, y) : %f , %f", peopleData[innerIndex].xdistance, peopleData[innerIndex].ydistance);
        ROS_INFO("P1 P2 interpersonal distance: %f",p1p2Distance);
        ROS_INFO("P1 distance from pr2 : %f", peopleData[index].distanceFromPR2);
        ROS_INFO("P2 distance from pr2 : %f", peopleData[innerIndex].distanceFromPR2);
        ROS_INFO("Time : %f   %f", currentTime, startTime );
        
        //To make the file consistent with person
        if (peopleFlag == 0){
          if(peopleData[index].distanceFromPR2 < peopleData[innerIndex].distanceFromPR2){
            firstPersonNo = 1;
            peopleFlag = 1;
          }
          else{
            firstPersonNo = 2;
            peopleFlag = 1;
          }
        } 
        ROS_INFO("First Person : %d", firstPersonNo );
        
        if(firstPersonNo == 1){
          outfile << timeStamp << ", "<< p1HallwayL << ", " << p1HallwayR << ", " << peopleData[index].distanceFromPR2 << ", "<< p2HallwayL << ", " << p2HallwayR << ", " << peopleData[innerIndex].distanceFromPR2 << ", "<< p1p2Distance  << endl;
          testfile << timeStamp << " , "<< m  << " ,  "<< c << peopleData[index].personId.c_str() << ", "<< " ,  "<< p1HallwayL << " ,  " << p1HallwayR <<  " ,  " << peopleData[index].xdistance << " ,  " << peopleData[index].ydistance << " , " << peopleData[index].distanceFromPR2 << " , "<< peopleData[innerIndex].personId.c_str() << " , "<< p2HallwayL << " , " << p2HallwayR << " , " << peopleData[innerIndex].xdistance << " , " << peopleData[innerIndex].ydistance << " , " << peopleData[innerIndex].distanceFromPR2 <<" , "<< p1p2Distance  << endl;
        }
        else if(firstPersonNo == 2){
          outfile << timeStamp << ", "<< p2HallwayL << ", " << p2HallwayR << ", " << peopleData[innerIndex].distanceFromPR2 << ", "<< p1HallwayL << ", " << p1HallwayR << ", " << peopleData[index].distanceFromPR2 << ", "<< p1p2Distance  << endl;
          testfile << timeStamp << " , "<< m  << " ,  "<< c << peopleData[innerIndex].personId.c_str() << ", "<< " ,  "<< p2HallwayL << " ,  " << p2HallwayR <<  " ,  " << peopleData[innerIndex].xdistance << " ,  " << peopleData[innerIndex].ydistance << " , " << peopleData[innerIndex].distanceFromPR2 << " , "<< peopleData[index].personId.c_str() << " , "<< p1HallwayL << " , " << p1HallwayR << " , " << peopleData[index].xdistance << " , " << peopleData[index].ydistance << " , " << peopleData[index].distanceFromPR2 <<" , "<< p1p2Distance  << endl;
        }
      }
    }
    
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

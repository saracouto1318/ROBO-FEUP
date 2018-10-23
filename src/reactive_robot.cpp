# include "../include/reactive_robot.h"

using namespace std;

#define PI 3.14159265358979323846

namespace reactive_robot
{
	WallFollowing::WallFollowing(int argc, char **argv)
	{
		if(argc != 3)
		{
			ROS_ERROR(
			"Usage : reactive_robot <robot_id> <laser_id>");
			exit(0);
		}

    laser_topic_ = string("/") + string(argv[1]) + string("/") + string(argv[2]);
    speeds_topic_ = string("/") + string(argv[1]) + string("/cmd_vel");

    subscriber_ = n_.subscribe(laser_topic_.c_str(), 1, &WallFollowing::callback, this);

    cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>(speeds_topic_.c_str(), 1);
	}


	WallFollowing::~WallFollowing(void){}

  void WallFollowing::callback(const sensor_msgs::LaserScan& msg)
	{
	    scan_ = msg;
    	geometry_msgs::Twist cmd;

      float distanceRightSide = numeric_limits<float>::max();
      float distanceLeftSide = numeric_limits<float>::max();
      float distanceFront = numeric_limits<float>::max();
      float robotAngle = 0.0;

    	for(int i = 0; i < scan_.ranges.size(); i++){
			     float realDistance = scan_.ranges[i];
			     float sensorAngle = scan_.angle_min + (scan_.angle_increment * (180/PI)) * i;

           if(sensorAngle >= -120 && sensorAngle <= -30){
   			       if(realDistance < distanceRightSide){
                   robotAngle = sensorAngle;
   					       distanceRightSide = realDistance;
   				     }
     			  }
     			  else if(sensorAngle > 30 && sensorAngle <= 120){
              if(realDistance < distanceLeftSide){
                    robotAngle = sensorAngle;
                    distanceLeftSide = realDistance;
              }
            }
            else if(sensorAngle >= -30 && sensorAngle <= 30){
  			       if(realDistance < distanceFront)
  					       distanceFront = realDistance;
      			}
     	  }

     		float minDistance = min(distanceRightSide, distanceLeftSide);
     		//float alpha = 90.0 - abs(robotAngle);

     		if(minDistance <= scan_.range_max){
          //cout << "******** WALKING *********" << endl;
     			//cout << "DISTANCE TO WALL: " << minDistance << endl;
     			//cout << "ALPHA: " << alpha << endl;

     			cmd.linear.x = 0.3;
     			//https://www.seas.upenn.edu/sunfest/docs/papers/12-bayer.pdf
     			cmd.angular.z = 0.4;
          //(-20 * (sin((alpha * (PI/180)) - (minDistance - 1.5))) * cmd.linear.x;
     		}
     		else{
     			cmd.linear.x = 0.3;
     			cmd.angular.z = 0.1;
     		}
        /*
     		if(distanceFront <= 1.5 && distanceRightSide < scan_.range_max && distanceLeftSide < scan_.range_max) // on the pipe
     		{
     			cout << "******** STOPPED *********" << endl;
     			cout << "FRONT DISTANCE: " << distanceFront << endl;
     			cout << "LEFT DISTANCE: " << distanceLeftSide << endl;
     			cout << "RIGHT DISTANCE: " << distanceRightSide << endl;
     			cmd.linear.x = 0.0;
     			cmd.angular.z = 0.0;
     		}*/

  		cmd_vel_pub_.publish(cmd);
  	}

}

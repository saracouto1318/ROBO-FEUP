#include <stdlib.h>
#include <stdio.h>
#include "../include/reactive_robot/reactive_robot.h"

using namespace std;

#define PI 3.14159265358979323846

namespace reactive_robot
{
WallFollowing::WallFollowing(int argc, char **argv)
{
	if (argc != 3)
	{
		ROS_ERROR(
			"Usage : reactive_robot robot <robot_id> <laser_id>");
		exit(0);
	}

	laser_topic_ = string("/") + string(argv[1]) + string("/") + string(argv[2]);
	speeds_topic_ = string("/") + string(argv[1]) + string("/cmd_vel");

	subscriber_ = n_.subscribe(laser_topic_.c_str(), 1, &WallFollowing::callback, this);

	cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>(speeds_topic_.c_str(), 1);
}

WallFollowing::~WallFollowing(void) {}

void writeToFile(char* filename, char* str)
{
	FILE *f = fopen(filename, "a");
	if (f == NULL)
	{
		printf("Error opening file!\n");
		exit(1);
	}

	/* print some text */
	fprintf(f, "%s", str);

	fclose(f);
}

void WallFollowing::callback(const sensor_msgs::LaserScan &msg)
{
	scan_ = msg;
	geometry_msgs::Twist cmd;
	double startTime = time(0);
	//string filename = string("logs-") + startTime + "txt";
	char filename[] = "logs.txt";

	float distanceRightSide = numeric_limits<float>::max();
	float distanceLeftSide = numeric_limits<float>::max();
	float distanceFront = numeric_limits<float>::max();
	float robotAngle = 0.0;

	for (int i = 0; i < scan_.ranges.size(); i++)
	{
		float realDistance = scan_.ranges[i];
		float sensorAngle = (scan_.angle_min * (180 / PI)) + (scan_.angle_increment * (180 / PI)) * i;

		if (sensorAngle >= -120 && sensorAngle <= 0)
		{
			if (realDistance < distanceRightSide)
			{
				robotAngle = sensorAngle;
				distanceRightSide = realDistance;
			}
		}
		else if (sensorAngle > 0 && sensorAngle <= 120)
		{
			if (realDistance < distanceLeftSide)
			{
				robotAngle = sensorAngle;
				distanceLeftSide = realDistance;
			}
		}
	}

	float minDistance = min(distanceRightSide, distanceLeftSide);

	/**const float MAX_SPEED = 6.5;
	const float MIN_SPEED = 1.0;*/
	const float GOAL_DISTANCE = 1.5;
	const float K = -15;

	// factor up to 1 to mod speed according to wall distance, where 1 is perfect conditions
	//float distanceFactor = min(minDistance, GOAL_DISTANCE)/GOAL_DISTANCE;

	// values for angular speed
	float alpha = 90.0 - abs(robotAngle);
	

	cout << "Min Distance: " << minDistance << " Scan Range: " << scan_.range_max << endl;

	if (minDistance <= scan_.range_max)
	{
		cout << "******** WALKING *********" << endl;
		cout << "DISTANCE TO WALL: " << minDistance << endl;
		cout << "ALPHA: " << alpha << endl;

		/*cout << "Distance factor: " << distanceFactor << endl;
		cmd.linear.x = max(distanceFactor*MAX_SPEED, MIN_SPEED);*/
		cmd.linear.x = 0.5;

		//https://www.seas.upenn.edu/sunfest/docs/papers/12-bayer.pdf
		cmd.angular.z = (K * (sin(alpha * (PI / 180)) - (minDistance - GOAL_DISTANCE))) * cmd.linear.x;
	}
	else
	{
		//cmd.linear.x = MAX_SPEED;
		cmd.linear.x = 0.5;
		cmd.angular.z = 0.0;
	}

	cmd_vel_pub_.publish(cmd);

	double timeDiff = time(0) - startTime;
	char timeDiffStr[50];
	snprintf(timeDiffStr, 50, "%f", timeDiff);
	char minDistanceStr[20];
	snprintf(minDistanceStr, 20, "%f", minDistance);
	char alphaStr[20];
	snprintf(alphaStr, 20, "%f", alpha);
	char linearStr[20];
	snprintf(linearStr, 20, "%f", cmd.linear.x);
	char angularStr[20];
	snprintf(angularStr, 20, "%f", cmd.angular.z);

	char log[400];
	strcpy(log, "=================\ntime = ");
	strcat(log, timeDiffStr);
	strcat(log, "\nwallDistance = ");
	strcat(log, minDistanceStr);
	strcat(log, "\nalpha = ");
	strcat(log, alphaStr);
	strcat(log, "\nSpeed:\n    linear = ");
	strcat(log, linearStr);
	strcat(log, "\n    angular = ");
	strcat(log, angularStr);
	strcat(log, "\n\n");
	writeToFile(filename, log);
	cout << log;
}

} // namespace reactive_robot

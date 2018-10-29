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

void WallFollowing::callback(const sensor_msgs::LaserScan &msg)
{
	scan_ = msg;
	geometry_msgs::Twist cmd;

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
	const float K = -16;

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
}

} // namespace reactive_robot

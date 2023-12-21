#include "../stdafx.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>

double velocity_x = 0.0;
double velocity_y = 0.0;
double velocity_z = 0.0;

double arg_x      = 0.0;
double arg_y      = 0.0;
double arg_z      = 0.0;

double string_x_motion(double simvx, double windvx, double V)
{
	velocity_x   = simvx + windvx;
	return arg_x = -limit(velocity_x / 15, -1, 1);
}

double string_y_motion(double simvy, double windvy, double V)
{
	velocity_y   = simvy + windvy - 1/limit(V, 1, 999);
	return arg_y = -limit(velocity_y / 15, -1, 1);
}

double string_z_motion(double simvz, double windvz)
{
	velocity_z   = simvz + windvz;
	return arg_z = -limit(velocity_z / 15, -1, 1);
}

double sight_left_movement(double sightBackClick, double sightForwardClick, double LeftSightOff)
{
	if (LeftSightOff == 1)
	{
		return 2;
	}
	else
	{
		return 0;
	}
}

double sight_right_movement(double sightBackClick, double sightForwardClick, double RightSightOff)
{
	if (RightSightOff == 1)
	{
		return 2;
	}
	else
	{
		return 0;
	}
}
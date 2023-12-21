#include "../stdafx.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>

double calc_total_pressure(double q_airspeedGauge, double P)
{
	double P0 = P + q_airspeedGauge;
	return P0;
}

double calc_indicated_airspeed(double V_indicated_airspeed, double kgm3toslugft3, double P0, double P)
{
	double indicated_airspeed = pow((2 * (P0 - P)) / (.002377), .5);
	return indicated_airspeed;
}

double update_RPM_lights(double RPM)
{
	double state = 0.0;

	if (RPM >= 700 && RPM <= 2399)
	{
		state = 0.25;
	}
	else if (RPM > 2399 && RPM <= 2699)
	{
		state = 0.5;
	}
	else if (RPM > 2700 && RPM <= 3500)
	{
		state = 1;
	}
	else
	{
		state = 0;
	}
	return state;
}

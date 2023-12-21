#include "../stdafx.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>

double electricPump_psi_max     = 65; // psi (max fuel presure? able to handle takeoff)
double electricPump_psi_current = 0.0;

double headlight_state  = 0;
double brakeLight		= 0;
double hazzard_state	= 0;
double hazzard_timer	= 0;
double domelight_state  = 0;
double hook_target = 0;
double hook_pos = 0;

double updateHook(double power, double hookTarget)
{
	if (power)
	{
		if (hookTarget == 1 && hook_pos < 1)
		{
			hook_pos += 0.006;
			hook_pos = limit(hook_pos, 0, 1);
		}
		else if (hookTarget == 0 && hook_pos > 0)
		{
			hook_pos -= 0.006;
			hook_pos = limit(hook_pos, 0, 1);
		}
	}
	else if (!power && hookTarget > 0)
	{
		hook_pos -= 0.006;
		hook_pos = limit(hook_pos, 0, 1);
	}
	return hook_pos;
}
double update_electricPump(double electricPump)
{
	return electricPump_psi_current = electricPump;// *electricPump_psi_max;
}

double updateDomelights(double power, double dome)
{
	if (power && dome)
	{
		domelight_state = 0.5;
	}
	else { domelight_state = 0; };

	return domelight_state;
}
double updateHeadlights(double power, double headlight)
{
	if (power && headlight)
	{
		headlight_state = 1;
	}
	else { headlight_state = 0; };

	return headlight_state;
}
double updateBrakelight(double power, double brake)
{
	if (power && brake > 0)
	{
		brakeLight = 1;

	}
	else { brakeLight = 0; };

	return brakeLight;
}
double updateHazzard(double power, double hazzard, double dt)
{
	if (hazzard)
	{
		hazzard_timer += dt;
		//cout << hazzard_timer << endl;
	}
	if (hazzard_timer >= 1)
	{
		hazzard_timer = 0;
	}

	if (power && hazzard_timer >= 0.5)
	{
		hazzard_state = 1;
	}
	else { hazzard_state = 0; };

	return hazzard_state;
}
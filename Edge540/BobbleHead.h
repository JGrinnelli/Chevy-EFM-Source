#include "stdafx.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include <stdlib.h> 

bool pitch_add = false;
bool roll_add = false;

double bobble_pitch = 0;
double bobble_roll = 0;

double bobble_speed = 0.06;

double bobble_moment = 0;


double updateBobbleRoll()
{

	if (bobble_roll == 1)
	{
		roll_add = false;
	}
	else if (bobble_roll == -1)
	{
		roll_add = true;
	}

	if (!roll_add)
	{
		bobble_roll -= bobble_speed;
		bobble_roll = limit(bobble_roll, -1, 1);
	}
	else if (roll_add)
	{
		bobble_roll += bobble_speed;
		bobble_roll = limit(bobble_roll, -1, 1);
	}

	//cout << bobble_roll << endl;

	return bobble_roll;

}

double updateBobblePitch()
{

	if (bobble_pitch == 1)
	{
		pitch_add = false;
	}
	else if (bobble_pitch == -1)
	{
		pitch_add = true;
	}

	if (!pitch_add)
	{
		bobble_pitch -= bobble_speed;
		bobble_pitch = limit(bobble_pitch, -1, 1);
	}
	else if (pitch_add)
	{
		bobble_pitch += bobble_speed;
		bobble_pitch = limit(bobble_pitch, -1, 1);
	}

	//cout << bobble_pitch << endl;

	return bobble_pitch;

}
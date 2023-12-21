#include "stdafx.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include <stdlib.h>
#include "ED_FM_Utility.h"

double deploy_pos  = 0;
double chute_roll  = 0;
bool   para_roll  = false;
bool   para_pitch = false;
double chute_pitch = 0;

double updateDragChute(double handle, double ias)
{
	if (handle && ias >= 30)
	{
		deploy_pos += 0.02;
		deploy_pos = limit(deploy_pos, 0, 1);
	}

	if (deploy_pos > 0 && ias < 10)
	{
		deploy_pos = 0;
	}
	//cout << ias << endl;

	return deploy_pos;
}

double updateChuteRoll()
{
	if (deploy_pos > 0)
	{
		if (para_roll)
		{
			chute_roll += 0.1;
			chute_roll = limit(chute_roll, -0.5, 0.5);
		}
		else if (!para_roll)
		{
			chute_roll -= 0.1;
			chute_roll = limit(chute_roll, -0.5, 0.5);
		}
	}

	if (chute_roll  == 0.5)
	{
		para_roll = false;
	}
	else if (chute_roll == -0.5)
	{
		para_roll = true;
	}
	//cout << chute_roll << endl;

	return 0;// chute_roll;
}

double UpdateChutePitch()
{
	if (deploy_pos > 0)
	{
		if (para_pitch)
		{
			chute_pitch += 0.1;
			chute_pitch = limit(chute_pitch, -0.5, 0.5);
		}
		else if (!para_pitch)
		{
			chute_pitch -= 0.1;
			chute_pitch = limit(chute_pitch, -0.5, 0.5);
		}
	}

	if (chute_pitch == 0.5)
	{
		para_pitch = false;
	}
	else if (chute_roll == -0.5)
	{
		para_pitch = true;
	}
	//cout << chute_pitch << endl;

	return 0;// chute_pitch;
}
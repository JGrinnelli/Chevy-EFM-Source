#include "../stdafx.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>

double center_current = 0.0;
double wing_current = 0.0;
double enginePump_current = 0.0;

double srb_fuel;
double main_fuel;

double update_enginePump(double RPM)
{
	if (RPM > 20)
	{
		enginePump_current = 1.0;
	}
	else
	{
		enginePump_current = 0.0;
	}

	return enginePump_current;
}

double normalize_centerTank(double commanded, double range, double dependency, double tau)
{
	double error = commanded * dependency - center_current;
	center_current = tau * error + center_current;
	
	return center_current / range;
}

double normalize_wingTank(double commanded, double range, double dependency, double tau)
{
	double error = commanded * dependency - wing_current;
	wing_current = tau * error + wing_current;

	return wing_current / range;
}

double normalizeSRB(double fuel)
{
	//srb_fuel = (fuel - 59) / (159 - 59) * 100;
	srb_fuel = (fuel - 59) / 100 * 100;
	srb_fuel = limit((srb_fuel * 0.01),0,1);
	//cout << "SRB FUEL: " << srb_fuel << endl;
	return srb_fuel;
}

double normalizeMFT(double fuel)
{
	//srb_fuel = (fuel - 59) / (159 - 59) * 100;
	main_fuel = fuel / 159 * 100;
	main_fuel = limit((main_fuel * 0.01), 0, 1);
	//cout << "MFT FUEL: " << main_fuel << endl;
	return main_fuel;
}
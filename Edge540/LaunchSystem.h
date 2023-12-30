#include "stdafx.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include <stdlib.h> 

bool LaunchCountdown = false;
bool Ignition = false;

double countdown_timer = 1;
double thrust_final = 0;

double gear_pos	   = 0;
double gear_target = 0;

double theClock = 0;

double SRBarg = 0;
double RocketArg = 0;

double srb_jet = 0;
double main_jet = 0;

double flame_length_M = 0;
double flame_length_R = 0;

double flame_pulse_M  = 0;
double flame_pulse_R  = 0;


double SRB_ARMED = 0;
double MFT_ARMED = 0;
double MASTER_SW = 0;
double FUEL_SW   = 0;

bool PilotHasControl = false;
bool LaunchReady = false;

double Stage0 = 0;
double Stage1 = 0;
double Stage2 = 0;
double Stage3 = 0;
double Stage4 = 0;

double trim_bias = 0;

double smoke_SRB = 0;
double smoke_MAIN = 0;



void ColdLaunchSystem()
{
	LaunchCountdown = false;
	Ignition		= false;

	countdown_timer = 5;
	thrust_final	= 0;
	theClock		= 0;
	srb_jet			= 0;

	flame_length_M = 0;
	flame_length_R = 0;
	flame_pulse_M  = 0;
	flame_pulse_R  = 0;

	SRB_ARMED	   = 0;
	MFT_ARMED	   = 0;
	MASTER_SW	   = 0;
	FUEL_SW		   = 0;

	PilotHasControl = false;
	LaunchReady = false;

	trim_bias = 0;
	smoke_SRB = 0;
	smoke_MAIN = 0;
}

double updatetheClock(double dt)
{
	if (Ignition)
	{
		theClock += dt;
		//cout << "THE CLOCK IS RUNNING: " << theClock << endl;
	}

	return theClock;
}
double updateCountdown(double dt)
{
	if (LaunchCountdown)
	{
		countdown_timer -= dt;
		countdown_timer = limit(countdown_timer, 0, 10);
		cout << "LAUNCH COUNTDOWN: " << countdown_timer << endl;
	}

	if (countdown_timer == 0 && LaunchCountdown)
	{
		LaunchCountdown = false;
		Ignition = true;
		//cout << "LAUNCH COUNTDOWN STOP" << endl;
		cout << "LAUNCH IGNITION!" << endl;
	}

	return countdown_timer;
}

double updateGear()
{
	//Gear Logic
	if (gear_target == 1 && gear_pos < 0.98 && !Ignition)
	{
		gear_pos += 0.0002;
		gear_pos = limit(gear_pos, 0, 0.98);
		//std::cout << "LAUNCH POSITION: " << gear_pos << std::endl;
	}

	if (theClock > 3.5 && gear_pos > 0)
	{
		gear_pos = 0;
		//std::cout << "LAUNCH POSITION RETURN: " << gear_pos << std::endl;
	}
	return gear_pos;
}
void updateLaunchStatus()
{
	if (MASTER_SW && FUEL_SW && SRB_ARMED && MFT_ARMED)
	{
		LaunchReady = true;
	}
	else
	{
		LaunchReady = false;
	}
}
void updateRocketStage()
{
	if (Ignition)
	{
		if (theClock < 3)
		{
			Stage0 = true; // Sound and Fire No Thrust!!!!
			RocketArg = 1;
			SRBarg = 1;
			//cout << "STAGE 0" << endl;
		}
		else if (theClock >= 3 && theClock < 40)
		{
			Stage0 = false;
			Stage1 = true; // Initial Thrust all engines
			RocketArg = 1;
			SRBarg = 1;
		}
		else if (theClock >= 40 && theClock < 90)
		{
			srb_jet = 1;
			Stage1 = false;
			Stage2 = true; // Main Thrust ONLY
			RocketArg = 1;
			SRBarg = 0;

		}
		else if (theClock >= 90)
		{
			main_jet = 1;
			Stage2 = false;
			Stage3 = true; // NO THRUST
			RocketArg = 0;
			PilotHasControl = true;
		}

	}


}

double updateRocket()
{
	//if (Stage1)
	//{
	//	thrust_final += 50;
	//	thrust_final = limit(thrust_final, 0, 30000);
	//}
	//else if (Stage2)
	//{
	//	thrust_final += 50;
	//	thrust_final = limit(thrust_final, 0, 30000);
	//}
	//else if (Stage3)
	//{
	//	thrust_final -= 50;
	//	thrust_final = limit(thrust_final, 0, 100000);
	//}

	//if (Ignition)
	//{
	//	thrust_final = 30000;
	//}

	//ORIGINAL PROFILE
	
	if (Ignition && theClock < 1)
	{
		thrust_final = 0; //35k=45 ft
		RocketArg = 1;
		SRBarg = 1;
		trim_bias = 0;
		//std::cout << "THRUST: " << thrust_final << std::endl;
	}
	else if (Ignition && theClock >= 1 && theClock < 3)
	{
		thrust_final = 30000; //35k=45 ft
		RocketArg = 1;
		SRBarg = 1;
		trim_bias = 0;
		smoke_SRB = 1;
		smoke_MAIN = 1;
		//std::cout << "THRUST: " << thrust_final << std::endl;
	}
	else if (Ignition && theClock >= 3 && theClock < 30)
	{
		thrust_final = 30000;//was 25000;
		RocketArg = 1;
		SRBarg = 1;
		trim_bias = 0.0005;
		//std::cout << "THRUST: " << thrust_final << std::endl;
	}
	else if (Ignition && theClock >= 30 && theClock < 50)
	{
		thrust_final = 25000; //was 15000
		RocketArg = 1;
		SRBarg = 0;
		srb_jet = 1;
		trim_bias = -0.0070;
		smoke_SRB = 1;
		//std::cout << "THRUST: " << thrust_final << std::endl;
	}
	else if (Ignition && theClock >= 50 && theClock < 51)
	{ 
		thrust_final = 0; 
		RocketArg = 0; 
		main_jet = 1;
		PilotHasControl = true;
		trim_bias = 0;
		smoke_SRB = 0;
	}
	

	//std::cout << "THRUST: " << thrust_final << std::endl;
	return thrust_final;
}

double updateFlameLengthMain(double dt)
{
	if (Ignition && theClock < 50)
	{
		//cout << "MAIN BURNER: " << MainBurner << endl;
		flame_length_M += 10 * dt;
		flame_length_M = limit(flame_length_M, 0.0, 0.25);
		//cout << "FLAME: " << flame_length_M << endl;
	}
	else if (theClock >= 50)
	{
		flame_length_M -= 15 * dt;
		flame_length_M = limit(flame_length_M, 0.0, 1.0);
	}
	
	return flame_length_M;
}
double updateFlamePulseMain(double dt)
{
	if (Ignition && theClock < 50)
	{
		//cout << "MAIN BURNER: " << MainBurner << endl;
		flame_pulse_M += 25 * dt;
		if (flame_pulse_M > 1)
		{
			flame_pulse_M = 0;
		}
	}
	else if (Ignition && theClock >= 50)
	{
		flame_pulse_M = flame_pulse_M;
	}
	//cout << "PULSE: " << flame_pulse << endl;
	return flame_pulse_M;
}
double updateFlameLengthSRB(double dt)
{
	if (Ignition && theClock < 30)
	{
		//cout << "MAIN BURNER: " << MainBurner << endl;
		flame_length_R += 10 * dt;
		flame_length_R = limit(flame_length_R, 0.0, 0.25);
		//cout << "SRB ON" << endl;
	}
	else if (Ignition && theClock >= 30)
	{
		flame_length_R -= 15 * dt;
		flame_length_R = limit(flame_length_R, 0.0, 1.0);
		//cout << "SRB OFF" << endl;
	}
	//cout << "FLAME: " << flame_length << endl;
	return flame_length_R;
}
double updateFlamePulseSRB(double dt)
{
	if (Ignition && theClock < 30)
	{
		//cout << "MAIN BURNER: " << MainBurner << endl;
		flame_pulse_R += 25 * dt;
		if (flame_pulse_R > 1)
		{
			flame_pulse_R = 0;
		}
	}
	else if (Ignition && theClock >= 30)
	{
		flame_pulse_R = flame_pulse_R;
	}
	//cout << "PULSE: " << flame_pulse << endl;
	return flame_pulse_R;
}

void LaunchDebug()
{
	//cout << "THE CLOCK IS RUNNING: " << theClock << "    THRUST: "<< thrust_final << endl;
}
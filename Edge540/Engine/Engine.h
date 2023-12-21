#include "../stdafx.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include <stdlib.h> 

double PropPitch		   = 0.0;
double idleRPM			   = 0.0;             // Lowest possible RPM of engine running at idle
double throttleRPM		   = 0.0;             // RPM produced by throttle
double propRPM			   = 0.0;             // RPM produced by prop
double airspeedRPM		   = 0.0;             // RPM produced by airspeed
double RPM_Current		   = 0.0;             // Current RPM
double RPM_Commanded	   = 0.0;			  // RPM Commanded
double RPM_Error		   = 0.0;             // Error between commanded and actual
double tau				   = 0.007;            // Delay for spool time
						   
double maniPress_Current   = 0.0;             // Manifold pressure
double mani_Commanded	   = 0.0;             // Manifold pressure commanded
double maniTau			   = 0.01;           // Delay for manifold pressure
double mani_Error		   = 0.0;
						   
double fuelFlowmin_Current = 0.0;
double fuelFlowact_Current = 0.0;
double fuelFlow_Commanded  = 0.0;
double fuelFlowTau		   = 0.02;
double fuelFlow_Error      = 0.0;
						   
double stroke			   = 4.375;			  // Stroke (in)
double bore				   = 2.6595;		  // Bore radius (in)
double compress_ratio	   = 8.9;
double bore_area		   = M_PI * bore * bore;
double single_displacement = bore_area * stroke;
double total_displacement  = 0.0;			  // calculation in function
double energy_constant	   = 33000;
double tourque_constant    = 5252;
double preignition_press   = 0.0;
double MEPadjust		   = 0.0;             // Theoretical calculated adjustment value (units, inconsistency etc)
double MEP				   = 0.0;
double power_Current       = 0.0;
double isprimed            = 0.0;

double thrust_Current      = 0.0;

double prop = -1;
double propon = 0;
double blur = 0;

double FF_bhp[] = {
	0,
	177,
	207,
	272,
	280,
	320,
};

double FF[] = {
	0,
	93,
	99,
	135,
	138,
	143,
};

double nu[] = {
	.7,
	.8,
	.82,
	.83,
	.85,
	.97,
	.93,
	.8
};

double nu_pitch[] = {
	0,
	.2,
	.4,
	.5,
	.6,
	.75,
	.9,
	1,
};

double update_RPM(double V, double throttle, double PropPitch, double fpstoknts, double mixture, double fuelFlowMin, double fuelFlow, double enginePump, double electricBoost, double cyl_fuel, double MagState, double isFeed, double Mani)
{
	//throttleRPM   = (1900) * (limit(V * fpstoknts, 0, 100) / 100) + (throttle * 1800) * (1 - limit(V * fpstoknts, 0, 100) / 100);
	//propRPM       = -(PropPitch * 1500);
	//airspeedRPM   = V * fpstoknts;

	throttleRPM = 2946.7 * limit(Mani, 0, 20) / 39.285 - ((20 - limit(Mani, 0, 20)) * 102);
	propRPM = (-PropPitch + 1) * 1249.9 + (-20 + limit(Mani, 0, 20)) * 27.493;
	airspeedRPM = (rand() % 100 + 1) / 100;

	if ((fuelFlow >= fuelFlowMin) && (MagState == 0.25 || MagState == 0.5 || MagState == 0.75) && (enginePump == 1 || electricBoost == 1) && isFeed == 1)
	{
		if (MagState == 0.75)
		{
			RPM_Commanded = limit(idleRPM + throttleRPM + propRPM + airspeedRPM, 670, 2800) * enginePump;
		}
		else if (MagState == 0.5)
		{
			RPM_Commanded = limit(idleRPM + throttleRPM + propRPM + airspeedRPM, 670, 2800) * enginePump - throttle * 1.5 * 115;
		}
		else if (MagState == 0.25)
		{
			RPM_Commanded = limit(idleRPM + throttleRPM + propRPM + airspeedRPM, 670, 2800) * enginePump - throttle * 1.5 * 135;
		}

		RPM_Error = RPM_Commanded - RPM_Current;
		RPM_Current = RPM_Error * tau + RPM_Current;
	}
	else if ((fuelFlow < fuelFlowMin) && (MagState == 0.25 || MagState == 0.5 || MagState == 0.75) && (enginePump == 1 || electricBoost == 1) && isFeed == 1)
	{
		RPM_Commanded = limit(idleRPM + throttleRPM + propRPM + airspeedRPM, 670, 2800) * enginePump * mixture;

		RPM_Error = RPM_Commanded - RPM_Current;
		RPM_Current = RPM_Error * tau + RPM_Current;
	}
	else if (MagState == 1 && (electricBoost == 1 || electricBoost == 0) && cyl_fuel > 1.2 && isFeed == 1)
	{
		RPM_Commanded = 700;

		RPM_Error = RPM_Commanded - RPM_Current;
		RPM_Current = RPM_Error * 0.006 + RPM_Current;
	}
	else
	{
		RPM_Commanded = 0;

		RPM_Error = RPM_Commanded - RPM_Current;
		RPM_Current = RPM_Error * tau + RPM_Current;
	}

	
	return RPM_Current;
}

double update_Mani(double P, double psftoinHg, double throttle, double RPM)
{
	/*if (RPM > 700)
	{
		mani_Commanded = P * psftoinHg + (P * psftoinHg - 10) * (throttle - 1);
	}
	else if (RPM <= 700)
	{
		mani_Commanded = P * psftoinHg + (P * psftoinHg - 10) * (throttle - 1) * limit(RPM / 2700, 0, 1);
	}*/

	mani_Commanded = P * psftoinHg + (P * psftoinHg - 10) * (throttle - 1) * limit((RPM-700), 0, 1);
	mani_Error        = mani_Commanded - maniPress_Current;
	maniPress_Current = mani_Error * maniTau + maniPress_Current;
	return maniPress_Current;
}

double update_Mani_gauge(double Mani)
{
	double maniPressGaugeArg = ((Mani - 10) / 25);
	return maniPressGaugeArg;
}

double update_fuelFlow_min(double RPM, double maniPress, double power)
{
	fuelFlowmin_Current = (lerp(FF_bhp, FF, sizeof(FF_bhp) / sizeof(double), power) / 6.01);
	return fuelFlowmin_Current;
}

double update_fuelFlow_actual(double fuelFlow_min, double Mixture, double elctrciBoostPump, double RPM, double Master, double isFeed)
{
	double fuelFlow_actual;
	if (RPM < 400 && elctrciBoostPump == 1 && Mixture > .5 && isFeed == 1 && Master == 1)
	{
		fuelFlow_actual = 8;
	}
	else
	{
		fuelFlow_actual = fuelFlow_min * Mixture * 1.2;
	}

	fuelFlow_Error = fuelFlow_actual - fuelFlowact_Current;
	fuelFlowact_Current = fuelFlowTau * fuelFlow_Error + fuelFlowact_Current;
	return fuelFlowact_Current;
}

double update_fuelFlow_gauge(double fuelFlow)
{
	double fuelFlowGaugeArg = 0.0;
	return fuelFlowGaugeArg = fuelFlow / 35;
}

double update_prop(double RPM, double MagState, double Master, double mixture)
{

	if (MagState == 1 && Master == 1 && RPM <= 80)
	{
		prop = prop + (.006 / 0.6); //0.5
		//std::cout << "SPIN NO FUEL" << std::endl;
		if (prop >= 0)
		{
			prop = -1;
		}
		return prop;
	}
	else if ((MagState == 1 || MagState == .75 || MagState == .5 || MagState == .25) && RPM > 80)//500
	{
		propon = propon + (.006 / (50 / RPM));
		//std::cout << "RUNNING" << std::endl;
		if (propon >= 1)
		{
			propon = 0; // 0.1
		}
		return propon;
	}
	else if (MagState == 0 && RPM > 80 && mixture == 0)
	{
		propon = propon + (.006 / 0.6);
		if (propon >= 1)
		{
			propon = 0; // 0.1
		}
		//std::cout << "SHUTDOWN: " << std::endl;
		return propon;
	}
	else if (MagState == 0 && RPM > 80)
	{
		propon = propon + (.006 / 0.6);
		if (propon >= 1)
		{
			propon = 0; // 0.1
		}
		//std::cout << "SHUTDOWN: " << std::endl;
		return propon;
	}
	else
	{
		//std::cout << "RETURN" << std::endl;
		return prop;
	}

}

double update_blur(double RPM)
{
	if (RPM < 100)
	{
		blur = 0;
		return blur;
	}
	
	else if (RPM >= 101 && RPM < 695)
	{
		blur = 0.25;
		return blur;
	}
	else if (RPM >= 695 && RPM < 1000)
	{
		blur = 0.50;
		return blur;
	}
	else if (RPM >= 1000 && RPM < 2000)
	{
		blur = 0.75;
		return blur;
	}
	else if (RPM >= 2000 && RPM < 90000)
	{
		blur = 0.90;
		return blur;
	}
}

double update_nuProp(double pitch)
{
	return lerp(nu_pitch, nu, sizeof(nu_pitch) / sizeof(double), pitch);
}

// START OF ENGINE MODEL //

double is_prime(double engineFuelPump, double electricBoostPump, double Mixture)
{

	if ((engineFuelPump == 1 || electricBoostPump == 1) && Mixture > .3)
	{
		isprimed = limit(isprimed + Mixture * .01, 0, 25);
	}
	else
	{
		isprimed = limit(isprimed - .0007, 0, 25);
	}
	
	return isprimed;
}

double update_cyl1()
{
	double cyl_heat;
	double cyl_fuel;   // Is fuel introduced into the mixture during this update cycle?
	double cyl_damage; // Pre ignition or detonation, 1 good, 0 bad
	double cyl_state;  // State of cylinder between 0-1


	cyl_fuel = 1; // for now

	cyl_damage = 1; // for now
	cyl_heat = 1; // for now

	return cyl_state = cyl_fuel * cyl_damage * cyl_heat;
}

double update_cyl2()
{
	double cyl_heat;
	double cyl_fuel;   // Is fuel introduced into the mixture during this update cycle?
	double cyl_damage; // Pre ignition or detonation, 1 good, 0 bad
	double cyl_state;  // State of cylinder between 0-1

	cyl_fuel = 1; // for now
	cyl_damage = 1; // for now
	cyl_heat = 1; // for now

	return cyl_state = cyl_fuel * cyl_damage * cyl_heat;
}

double update_cyl3()
{
	double cyl_heat;
	double cyl_fuel;   // Is fuel introduced into the mixture during this update cycle?
	double cyl_damage; // Pre ignition or detonation, 1 good, 0 bad
	double cyl_state;  // State of cylinder between 0-1

	cyl_fuel = 1; // for now
	cyl_damage = 1; // for now
	cyl_heat = 1; // for now

	return cyl_state = cyl_fuel * cyl_damage * cyl_heat;
}

double update_cyl4()
{
	double cyl_heat;
	double cyl_fuel;   // Is fuel introduced into the mixture during this update cycle?
	double cyl_damage; // Pre ignition or detonation, 1 good, 0 bad
	double cyl_state;  // State of cylinder between 0-1

	cyl_fuel = 1; // for now
	cyl_damage = 1; // for now
	cyl_heat = 1; // for now

	return cyl_state = cyl_fuel * cyl_damage * cyl_heat;
}

double update_cyl5()
{
	double cyl_heat;
	double cyl_fuel;   // Is fuel introduced into the mixture during this update cycle?
	double cyl_damage; // Pre ignition or detonation, 1 good, 0 bad
	double cyl_state;  // State of cylinder between 0-1

	cyl_fuel = 1; // for now
	cyl_damage = 1; // for now
	cyl_heat = 1; // for now

	return cyl_state = cyl_fuel * cyl_damage * cyl_heat;
}

double update_cyl6()
{
	double cyl_heat;
	double cyl_fuel;   // Is fuel introduced into the mixture during this update cycle?
	double cyl_damage; // Pre ignition or detonation, 1 good, 0 bad
	double cyl_state;  // State of cylinder between 0-1

	cyl_fuel = 1; // for now
	cyl_damage = 1; // for now
	cyl_heat = 1; // for now

	return cyl_state = cyl_fuel * cyl_damage * cyl_heat;
}

double update_power(double mani, double P, double RPM, double T, double electricBoostPump, double engineFuelPump, double Mixture)
{
	total_displacement   = (update_cyl1() + update_cyl2() + update_cyl3() + update_cyl4() + update_cyl5() + update_cyl6()) * single_displacement;
	preignition_press    = (mani * compress_ratio) + (P - 29.92);
					     
	MEPadjust            = 20.17158 / (0.3154 * pow(mani, 0.3502));
	MEP                  = preignition_press / MEPadjust;

	return power_Current = limit((MEP * total_displacement * (RPM / 2)) / ((energy_constant) * pow((475) / (460 + T), .5)), 0, 327);
	//return power_Current = limit((MEP * total_displacement * (RPM / 2)) / ((energy_constant)*pow((975) / (460 + T), .5)), 0, 327);
}

double update_thrust(double rho, double power, double prop_disc_area)
{
	return thrust_Current = pow(power * power * 746 * 746 * 4 * rho * prop_disc_area / 10.764, 0.333);
	//return thrust_Current = pow(power * power * 1500 * 1500 * 15 * rho * prop_disc_area / 2, 0.333);
}
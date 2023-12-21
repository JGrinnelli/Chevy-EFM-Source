#include "../stdafx.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
				     
double PropPitch          = 0.0;
double idleRPM            = 650;             // Lowest possible RPM of engine running at idle
double throttleRPM        = 0.0;             // RPM produced by throttle
double propRPM            = 0.0;             // RPM produced by prop
double airspeedRPM        = 0.0;             // RPM produced by airspeed
double RPM_Current        = 0.0;             // Current RPM
double RPM_Commanded      = 0.0;			    // RPM Commanded
double RPM_Error          = 0.0;             // Error between commanded and actual
double tau                = 0.01;            // Delay for spool time
					      
double maniPress_Current  = 0.0;             // Manifold pressure
double mani_Commanded     = 0.0;             // Manifold pressure commanded
double maniTau            = 0.055;           // Delay for manifold pressure
double mani_Error         = 0.0;

double fuelFlow_Commanded = 0.0;
double fuelFlow_Current   = 0.0;
double fuelFlow_Error     = 0.0;
double fuelFlowTau        = 1;

double bhp_Commanded      = 0.0;
double bhp_Current        = 0.0;
double bhp_Error          = 0.0;
double bhpTau             = .007;

double FF_2000_mani[] = {
	20,
	29.5,
};

double FF_2000_bhp[] = {
	135,
	230,
};

double FF_2200_mani[] = {
	19.2,
	29.5,
};

double FF_2200_bhp[] = {
	144,
	259,
};

double FF_2400_mani[] = {
	18.6,
	29.25,
};

double FF_2400_bhp[] = {
	147.5,
	289,
};

double FF_2500_mani[] = {
	18.49,
	29.23,
};

double FF_2500_bhp[] = {
	150.5,
	297,
};

double FF_2700_mani[] = {
	17.9,
	29.1,
};

double FF_2700_bhp[] = {
	154.2,
	315,
};

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

double update_RPM(double V, double throttle, double PropPitch, double fpstoknts)
{
	throttleRPM   = (1900) * (limit(V * fpstoknts, 0, 100) / 100) + (throttle * 1800) * (1 - limit(V * fpstoknts, 0, 100) / 100);
	propRPM       = -(PropPitch * 1500);
	airspeedRPM   = V * fpstoknts;

	RPM_Commanded = limit(idleRPM + throttleRPM + propRPM + airspeedRPM, 650, 2900);
	RPM_Error     = RPM_Commanded - RPM_Current;

	RPM_Current = RPM_Error * tau + RPM_Current;
	return RPM_Current;
}

double update_Mani(double P, double psftoinHg, double throttle)
{
	mani_Commanded    = (20 * throttle + 10);
	mani_Error        = mani_Commanded - maniPress_Current;
	maniPress_Current = mani_Error * maniTau + maniPress_Current;
	return maniPress_Current;
}

double update_Mani_gauge(double Mani)
{
	double maniPressGaugeArg = (Mani - 10) / 20;
	return maniPressGaugeArg;
}

double lexp(double * x, double * y, double xi)
{
	double dx = x[1] - x[0];
	double dy = y[1] - y[0];

	return ((dy / dx) * (xi - x[0]) + y[0]);
}

double interpolate(double RPM, double mani, double RPM_Low, double RPM_High, double * FF_mani_table_low, double * FF_bhp_table_low, double * FF_mani_table_high, double * FF_bhp_table_high)
{
	double resultLow = 0.0;
	double resultHigh = 0.0;
	double result = 0.0;

	double dbhp = 0.0;
	double drpm = 0.0;

	resultLow = lexp(FF_mani_table_low, FF_bhp_table_low, mani);
	resultHigh = lexp(FF_mani_table_high, FF_bhp_table_high, mani);
	dbhp = resultHigh - resultLow;
	drpm = RPM_High - RPM;
	result = resultLow + drpm / dbhp;

	return result;
}

double update_fuelFlow(double RPM, double maniPress, double T)
{
	double RPM_Low           = 0.0;
	double RPM_High          = 0.0;
	double table_mani_low[]  = { 0, 20};
	double table_bhp_low[]   = { 0, 135};
	double table_mani_high[] = { 0, 0};
	double table_bhp_high[]  = { 0, 0};

	if (RPM >= 0 && RPM < 2000)
	{
		RPM_Low           = 0;
		RPM_High          = 2000;
		for (int i = 0; i <= 1; i++)
		{
			table_mani_high[i] = FF_2000_mani[i];
			table_bhp_high[i]  = FF_2000_bhp[i];
		}
	}
	else if (RPM >= 2000 && RPM < 2200)
	{
		RPM_Low = 2000;
		RPM_High = 2200;

		for (int i = 0; i <= 1; i++)
		{
			table_mani_low[i]  = FF_2000_mani[i];
			table_bhp_low[i]   = FF_2000_bhp[i];
			table_mani_high[i] = FF_2200_mani[i];
			table_bhp_high[i]  = FF_2200_bhp[i];
		}

	} 
	else if (RPM >= 2200 && RPM < 2400)
	{
		RPM_Low = 2200;
		RPM_High = 2400;

		for (int i = 0; i <= 1; i++)
		{
			table_mani_low[i]  = FF_2200_mani[i];
			table_bhp_low[i]   = FF_2200_bhp[i];
			table_mani_high[i] = FF_2400_mani[i];
			table_bhp_high[i]  = FF_2400_bhp[i];
		}

	} 
	else if (RPM >= 2400 && RPM < 2500)
	{
		RPM_Low = 2400;
		RPM_High = 2500;

		for (int i = 0; i <= 1; i++)
		{
			table_mani_low[i]  = FF_2400_mani[i];
			table_bhp_low[i]   = FF_2400_bhp[i];
			table_mani_high[i] = FF_2500_mani[i];
			table_bhp_high[i]  = FF_2500_bhp[i];
		}

	}
	else if (RPM >= 2500 && RPM < 2775)
	{
		RPM_Low = 2500;
		RPM_High = 2775;

		for (int i = 0; i <= 1; i++)
		{
			table_mani_low[i]  = FF_2500_mani[i];
			table_bhp_low[i]   = FF_2500_bhp[i];
			table_mani_high[i] = FF_2700_mani[i];
			table_bhp_high[i]  = FF_2700_bhp[i];
		}

	}

	bhp_Commanded = interpolate(RPM, maniPress, RPM_Low, RPM_High, table_mani_low, table_bhp_low, table_mani_high, table_bhp_high) * pow((475)/(460+T),.5);
	bhp_Error = bhp_Commanded - bhp_Current;
	bhp_Current = bhp_Error * bhpTau + bhp_Current;
	fuelFlow_Commanded = lerp(FF_bhp, FF, sizeof(FF_bhp) / sizeof(double), bhp_Current);
	fuelFlow_Error = fuelFlow_Commanded - fuelFlow_Current;
	return fuelFlow_Current = (fuelFlow_Error * fuelFlowTau + fuelFlow_Current) / 6.01;
}

double update_fuelFlow_gauge(double fuelFlow)
{
	double fuelFlowGaugeArg = 0.0;
	return fuelFlowGaugeArg = fuelFlow / 35;
}
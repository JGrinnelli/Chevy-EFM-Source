#include "../stdafx.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>

double calc_lift(double CL, double A, double q)
{
	double L = CL * A * q;
	return L;
}

double calc_drag(double CD, double rho, double V, double A)
{
	double D = CD * A * rho * V * V  * 0.5;
	return D;
}

double calc_normal(double alpha, double L, double D)
{
	double N = L * cos(alpha) + D * sin(alpha);
	return N;
}

double calc_CDde(double CDde_const, double input, double elevator_deg, double alpha, double CL, double q, double A)
{
	double CDde = (abs(input) * elevator_deg) * (M_PI /180) * cos(alpha) * cos(alpha) * CDde_const * CL * CL * q * A;
	return CDde;
}

double calc_CD(double CD0, double CDbeta, double CDde, double CDi)
{
	double CD = CD0 + CDi + CDbeta + CDde;
	return CD;
}

double calc_drag(double q, double A, double CD)
{
	double D = CD * A * q;
	return D;
}

double calc_side(double A, double beta, double q, double CYb)
{
	double Y = q * A * sin(beta) * CYb * .5;
	return Y;
}


double calc_axial(double alpha, double L, double D)
{
	double A = -L * sin(alpha) + D * cos(alpha);
	return A;
}

double rollmoment(double Clb, double Clp, double Clda, double Cldr, double q_prop, double q, double c, double A, double latStickInput, double aileron_DEG, double rudder_DEG, double b, double beta, double roll_rate, double rho, double u, double aoa, double Clr, double V, double pedalInput, double yaw_rate, double angle_of_roll, double Vindicated, double longStickInput, double aoaRaw, double betaRaw)
{
	double Cldadot = (0.0000018 / limit(abs(.011 * roll_rate * (180 / M_PI)), 1, 420)) * pow(limit(aoaRaw, -10, 28), 2) - 0.001046;  // vary with roll rate
	double L  = q * A * b * Clb * sin(beta);																			            // Clb roll moment due to beta
	L += (Cldadot * roll_rate * rho * A * b * b * 0.0625) * pow((b * b * roll_rate * roll_rate) + (u * u * 16), .5);			    // Clp Roll moment due to roll rate extended to work at zero velocity
    L += avg(q_prop, .03, q, .997) * A * b * ((latStickInput / limit(abs(aoaRaw / 15), 1 , 1.7)) * aileron_DEG * (M_PI/180)) * Clda * cos(aoa)      * 400;	// Clda Roll moment due to aileron 315
    L += q * A * b * yaw_rate * Clr * (b / (2 * V))																            * 675;	// Clr Roll moment due to yaw rate 100
    L += q * A * b * (pedalInput * rudder_DEG * (M_PI/180)) * Cldr												           * 1300;	// Cldr Roll moment due to rudder 300
	//L += limit((pow(abs(aoa) * (1 / 1.9) * limit(beta * 180 / 3.14, -15, 15)* (3.14 / 180) * -(10000 * 180.0 / 3.14), 3) / 1200000000) * limit(Vindicated - 110, 0, 1) * limit(1 - abs(latStickInput), 0, 1) * limit(abs(longStickInput), 0, 1), -20000, 20000);
	L = limit(L, -5000000, 5000000);
	return L;
}

double pitchmoment(double Cmalpha, double Cmde, double Cmq, double Cmadot, double q_prop, double q, double q_x, double c, double A, double longStickInput, double elevator_DEG, double rudder_DEG, double b, double beta, double pitch_rate, double rho, double u, double aoa, double V, double pedalInput, double adot, double trim_bias, double WoW)
{
	double M = q * A * c * Cmalpha * sin(aoa)																	   * 66;	// Cmalpha Pitch moment due to alpha 60 * pitch_rate * (pitch_rate/abs(pitch_rate))
	
	
	if (WoW > 0)
	{
		M += q_prop * A * c * ((longStickInput + trim_bias) * elevator_DEG * (M_PI / 180)) * Cmde * 70.0;	// Cmde Pitch moment due to elevator 60
		M += (Cmq * pitch_rate * pitch_rate * pitch_rate * pitch_rate / abs(pitch_rate + 0.01) * pitch_rate * rho * A * c * c * 0.0625) * pow((c * c * pitch_rate * pitch_rate * pitch_rate * pitch_rate) + (u * u * 16), .5) * 6;// Cmq Pitch moment due to pitch rate extended to work at zero velocity (u * u * 16) -> 1638400 1.5
	}
	else {
		M += avg(q, .98, q_prop, .2) * A * c * ((longStickInput + trim_bias) * elevator_DEG * (M_PI / 180)) * Cmde * 55.0;
		M += (Cmq * pitch_rate * pitch_rate * pitch_rate * pitch_rate / abs(pitch_rate + 0.01) * pitch_rate * rho * A * c * c * 0.0625) * pow((c * c * pitch_rate * pitch_rate * pitch_rate * pitch_rate) + (u * u * 16), .5) * 2.5;// Cmq Pitch moment due to pitch rate extended to work at zero velocity (u * u * 16) -> 1638400 1.5
	}

	M += q * A * c * adot * Cmadot * (c / (2 * V))																   * 45;	// Cmadot Pitch moment due to alpha rate 20
	return M;
}

double yawmoment(double Cnb, double Cnr, double  Cndr, double Cnda, double q, double q_x, double c, double A, double latStickInput, double aileron_DEG, double rudder_DEG, double b, double beta, double yaw_rate, double rho, double u, double aoa, double V, double pedalInput)
{
	double N = q_x * A * b * Cnb * sin(beta)																		   * 150;	// Cnb Yaw moment due to beta 160
	N += (Cnr * yaw_rate * rho * A * b * b * 0.0625) * pow((b * b * yaw_rate * yaw_rate) + (u * u * 16), .5) * 2.1;// Cnr Yaw moment due to yaw rate extended to work at zero velocity
	N += cos(aoa) * q * A * b * (pedalInput* rudder_DEG * (M_PI / 180)) * Cndr					   		   * 130;	// Cndr Yaw moment due to rudder 430 (180 / V)
	N += -cos(aoa) * q * A * b * (latStickInput * aileron_DEG * (M_PI / 180)) * Cnda							   * 0;	// Cnda Adverse Yaw 20
	return N;
}
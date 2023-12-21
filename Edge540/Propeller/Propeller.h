#include "../stdafx.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>

double calc_advance_ratio(double V, double n, double D)
{
	double J = V / (n * D);
	return J;
};
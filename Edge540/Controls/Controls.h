#include "../stdafx.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>

enum inputs
{
	JoystickPitch = 2001,
	JoystickRoll = 2002,
	JoystickYaw = 2003,
	JoystickThrottle = 2004,

	LaunchPrep  = 3010,
	Countdown   = 3011,
	Master	    = 3012,
	FuelPump    = 3013,
	WheelBrake  = 3014,
	SRB_ARM	    = 3015,
	MFT_ARM     = 3016,
	Lights	    = 3017,
	Dome	    = 3018,
	Hazzard		= 3019,
	Key			= 3020,
	TrimUp		= 3021,
	TrimDown	= 3022,
	TrimNeutral = 3023,
	Hook		= 3024,
	Drag		= 3025,
	//Gear				= 68,
	//GearUp				= 430,
	//GearDown			= 431,



	//MagSelect			= 3050,
	//Master				= 3051,
	//BoostPump			= 3052,
	//FuelSelect			= 3053,
	//NavLights			= 3054,
	//FloodLights			= 3055,
	//GaugeLights			= 3056,
	//Routine				= 3057,

	//JoystickProp		= 3060,
	//JoystickMixture		= 3061,

	//NavPageSel			= 3070,
	//SysPageSel			= 3071,
	//MptPageSel			= 3072,
	//ConfigPageSel		= 3073,
	//WingletSel			= 3074,
	//SpadeSel			= 3075,
	//NightLightSel		= 3076,
	//WallSel				= 3077,
	//PilotSel			= 3078,
	//FloodSel			= 3079,

	//SightForward		= 3080,
	//SightBackward		= 3081,

	//TrimUp			= 10000,
	//TrimDown			= 10001,
	//TrimNeutral		= 10002,

	//SightBackward    = 10007,
	//SightForward     = 10008,

	//RSightOff       = 10009,
	//LSightOff       = 10010,

	//WheelBreakRight		= 15000,
	//WheelBreakLeft		= 15001,

};

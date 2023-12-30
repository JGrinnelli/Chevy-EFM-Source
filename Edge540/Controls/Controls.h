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

	//Start at 3010
	Key			= 3010,
	Shifter		= 3011,
	Blinker		= 3012,
	Hazzard		= 3013,
	WheelBrake  = 3014,
	Lights	    = 3015,
	Locks		= 3016,
	Dimmer		= 3017,

	Highbeams = 3018,
	BeamToggle = 3019,

	Throttle = 3020,
	TurnLeft = 3021,
	TurnRight = 3022,

	Window_FL = 3300,
	Window_FR = 3301,
	Window_RL = 3302,
	Window_RR = 3303,

	Door_FL = 3304,
	Door_FR = 3305,
	Door_RL = 3306,
	Door_RR = 3307,

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

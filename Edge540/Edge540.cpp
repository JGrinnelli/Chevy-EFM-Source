// ED_FM_Template.cpp : Defines the exported functions for the DLL application.
#include "stdafx.h"
#include "Edge540.h"
#include "ED_FM_Utility.h"
#include <Math.h>
#include <stdio.h>
#include <string>

//file i/o
#include <iostream>
#include <fstream>
using namespace std;

// Model Headers
#include "Controls/Controls.h"
#include "Aerodynamics/Aerodynamics.h"
#include "Engine/Engine.h"
#include "Cockpit/Cockpit.h"
#include "Fuel/Fuel.h"
#include "Electrical/Electrical.h"
#include "Propeller/Propeller.h"
#include "String and Sight/StringSights.h"
#include "LaunchSystem.h"
#include "BobbleHead.h"
//#include"Parachute.h";

// EDAPI 
#include "EDAPI.h"

Vec3	common_moment;
Vec3	common_force;
Vec3    center_of_gravity;
Vec3	wind;
Vec3	velocity_world_cs;

// Scaled inertias
double	    Mx						 = 0.005865;
double		My						 = 0.003632;
double		Mz						 = 0.00304019;

double		throttle				 = 0.0;
double		latStickInput			 = 0.0;
double		longStickInput		     = 0.0;
double		pedalInput				 = 0.0;

double		internal_fuel			 = 0;
double		fuel_consumption_since_last_time  = 0;
double		speed_of_sound			 = 320;
double		mach					 = 0;

// Conversions
double		pi						 = acos(-1.0);		// Pi (3.14159....)
double		radiansToDegrees		 = 180.0 / pi;		// Conversion factor from radians to degrees
double		meterToFoot				 = 3.28084;			// Meter to foot conversion factor
double		degreestorads			 = pi / 180;
double		fpstomps				 = 0.3048;
double		mpstofps				 = 3.28084;
double		kgm3toslugft3			 = 0.00194032;
double		lbtoN					 = 4.448222;
double		ftlbtonm				 = 1.36;
double		mps2tofps2				 = 3.2808399;
double		Nm2topsf			     = 0.020885;
double		fpstoknts				 = 0.592484;
double	    psftoinHg                = 1 / 70.726;

// Atmoshphere
double		V_freeStream			= 0.0;			   // Free stream velocity
double		V_propWind				= 0.0;			   // Free stream velocity
double		rho						= 0.0;			   // Desnity
double		q						= 0.0;			   // Dynamic Pressure
double      q_forwardDir            = 0.0;
double		T						= 0.0;			   // Temp (C)
													   
// Body												   
double		simvy					= 0.0;			   
double		simvz					= 0.0;			   
double		simvx					= 0.0;			   
double      simax					= 0.0;			   
double      simay					= 0.0;			   
double      simaz					= 0.0;			   
double      aoaRaw					= 0.0;			   // Raw AoA
double		aoaRad					= 0.0;			   
double		betaRaw					= 0.0;			   // Raw beta
double		betaRad					= 0.0;			   // Raw beta
double	    windvx					= 0.0;
double	    windvy					= 0.0;
double	    windvz					= 0.0;
													   
													   
double		inertia_Ix_KGM2			= 12874.0;		   // Reference moment of inertia (kg/m^2)
double		inertia_Iy_KGM2			= 75673.6;		   // Reference moment of inertia (kg/m^2)
double		inertia_Iz_KGM2			= 85552.1;		   // Reference moment of inertia (kg/m^2)
double		alpha_limit				= 0.0;			   // Angle of attack (deg)
double		aoaRad_limit		    = 0.0;
double      beta_limit				= 0.0;			   // Slideslip angle (deg)
double		rollRate_rps			= 0.0;			   // Body roll rate (rad/sec)
double      angle_of_roll = 0.0;                       // Angle of roll (deg)
double		pitchRate_rps		    = 0.0;			   // Body pitch rate (rad/sec)
double		yawRate_rps				= 0.0;			   // Body yaw rate (rad/sec)
double		u						= 0.0;			   // Forward Velocity
double		u_freestream			= 0.0;

double		WoW					    = 0.0;

// Lifting Surfaces
double		wingSpan				= 24.4;				// Extra wing-span (m)
double		wingArea				= 106.00;			// Extra wing area (m^2)
double		meanChord				= 4.03;				// Extra mean aerodynamic chord (m)

double		elevator_DEG			= 25;				// Elevator deflection (deg)
double		rudder_DEG			    = 30;				// Rudder  deflection (deg)
double		aileron_DEG	            = 30;				// Aileron deflection (deg)
//double		trim_bias				= 0.0;

// Prop
double      J                       = 0.0;              // Advance ratio
double      n                       = 0.0;              // Prop rotation speed
double      Di                      = 35 / 12;          // Prop diameter (ft)

// Aero forces
double		N						= 0.0;				// Normal force lift
double		A						= 0.0;				// Axial force
double		L						= 0.0;				// Body axis lift force
double		Y					    = 0.0;				//Side force
double		CL						= 0.0;				// Lift coeff

double	    thrust					= 0.0;
double	    thrust_propWind		    = 0.0;
double      prop_disc_area			= M_PI * pow(Di, 2); // Prop disc area (ft^2) 11.6 32 

double		D						= 0.0;				// Body axis drag force
double		CD						= 0.0;				// Drag coeff
double		CD0						= 0.0;				// Profile Drag coeff
double		CDi						= 0.015;			// Induced Drag coeff (Function of CL)
double		CDde					= 0.027;			// Drag due to Elevator Deflection
double		CDbeta					= 0.0;			    // Drag due to beta

double		CYb						= -0.7;				// Side force due to beta -2.2

double		AR = (wingSpan * wingSpan) / wingArea;		// Aspect Ratio

double		Cm						= 0.0;				// Pitch moment
double		Cmalpha					= -1.45 * Mz;
double		Cmde					= 1.4 * Mz;			// Pitch moment due to elevator 1.1
double		Cmq						= -1.466 * Mz;		// Pitch moment due to pitch rate extended to work at zero velocity
double		Cmadot					= -7 * Mz;			// Pitch moment due to alpha rate
double		alphadot				= 0.0;				// Time rate of change for alpha

double		Clda					= 0.125 * Mx;		// Roll moment due to ailerons
double		Cldr					= 0.0035 * Mx;		// Roll moment due to ailerons
double		Cldadot					= -0.4 * Mx;		// Roll moment due to roll rate
double		Clr						= 0.15 * Mx;        // Roll moment due to yaw rate
double	    Clb						= -0.1 * Mx;        // Roll moment due to beta

double		Cn						= 0.0;				// Yaw moment
double		Cnb						= -0.11 * My;       // Yaw moment due to beta
double		Cnr						= 0.1411 * My;		// Yaw moment due to yaw rate extended to work at zero velocity
double		Cndr					= -0.045 * My;		// Yaw moment due to rudder
double		Cnda				    = -0.01 * My;		// Adverse Yaw

double		M_pitch					= 0.0;
double		L_roll					= 0.0;
double		N_yaw					= 0.0;

// Breaks
double BrakeLeft					= 0.0;
double BrakeRight					= 0.0;

// Engine
double nuPropEff				    = 1.20;				// Double check this
double nuEngineEff                  = 1.0;
double thrust_aval                  = 5000; 

double power                        = 0.0;             // Power of engine (ft lbs / s)

double RPM							= 0.0;             // Current RPM

double maniPress                    = 0.0;             // Manifold pressure (inHg)
double maniPressGaugeArg            = 0.0;

double fuelFlow_min                 = 0.0;              // Fuel Flow MINIMUM REQUIRED (gal/hr)
double fuelFlow 					= 0.0;              // Fuel Flow (gal/hr)
double fuelFlowGaugeArg             = 0.0;

double Mixture                      = 1.0;

double MagState                     = 0.0;              // State of mag (off r l both) linked to key in cockpit

double cyl_fuel				        = 0.0;

double Q_tourque					= 0.0;

// Fuel
double wingTank				    	= 0.0;				// 31.7 Gal
double centerTank                   = 0.0;              // 10.8 Gal -> 26.7 usable
double smokeTank				    = 0.0;			    // 19.6 Kg, 6.1 Gal, 43.2 lb

double centerTankArg		        = 0.0;  
double wingTankArg				    = 0.0;

double isFeed					    = 0.0;				// is fuel feeding?

double FuelSelected				    = 0.0;

double engineFuelPump				= 0.0;              // 1 is on 0 is off

bool unlimited_fuel					= false;
double MixPos;
double MixAxis;

// Electric
double MasterOnOff				    = 0.0;				// 1 is on 0 is off
double ElectricBoostOnOff		    = 0.0;				// 1 is on 0 is off
double ElectricBoostpsi				= 0.0;

// Cockpit
double P0							= 0.0;				// Total Pressure
double P							= 0.0;				// Static Pressure
double q_airspeedGauge				= 0.0;
double indicated_airspeed		    = 0.0;
double V_indicated_airspeed			= 0.0;
double RPM_light_state              = 0.0;

double shake                        = 0.0;              // Shake amplitude (m?)

double prop_animation				= -1;
double blur_animation				= 0;

// Sights and String
double string_x					    = 0.0;
double string_y					    = 0.0;
double string_z					    = 0.0;

double sightPosition                = -0.55;

double RightSightOff                = 0;
double LeftSightOff                 = 0;

//NEW SHIT YO!!
// New Bullshit
//double gear_pos = 0;
//double gear_target = 0;

double zero = 0.0;
double one = 1.0;

double MASTER_battery_state = 0.0;
double PILOT_select = 0.0;

double NavOnOff = 0.0;
double FloodOnOff = 0.0;
double GaugeOnOff = 0.0;
double RoutineOnOff = 0.0;

double NAV_light_state = 0.0;
double FLOOD_light_state = 0.0;
double FLOOD_color_state = 0.0;
double GAUGE_light_state = 0.0;

double dome_light = 0;
double headlight = 0;
double hazzard_light = 0;
double Key_SW = 0;
double Hook_SW = 0;
double Drag_SW = 0;

double longStickInputArg = 0;
double latStickInputArg = 0;
double pedalInputArg = 0;

double radio_opacity = 0;

double radar_alt = 0;

// Params
//Instantiate API object
EDAPI API;

//Create a pointer to the Handle
//void * MAGSTATE_PARAM = API.getParamHandle("MAGSTATE_PARAM");
//void * RPM_PARAM = API.getParamHandle("RPM_PARAM");
void * SPRING_KEY_PARAM = API.getParamHandle("SPRING_KEY_PARAM");
void * MIXTURE_PARAM	= API.getParamHandle("MIXTURE_PARAM");
//double MIXTURE_NUM      = API.getParam("MIXTURE_PARAM");
void* SRB	 = API.getParamHandle("SRB");
void* ROCKET = API.getParamHandle("ROCKET");
void* SRB_JET = API.getParamHandle("SRB_JET");
void* MAIN_JET = API.getParamHandle("MAIN_JET");

void* SMOKE_SRB = API.getParamHandle("SMOKE_SRB");
void* SMOKE_MAIN = API.getParamHandle("SMOKE_MAIN");

void* RADALT = API.getParamHandle("RADALT");

void* ADIPITCH = API.getParamHandle("ADIPITCH");

void* FLAME_LENGTH_MAIN = API.getParamHandle("FLAME_LENGTH_MAIN");
void* FLAME_PULSE_MAIN  = API.getParamHandle("FLAME_PULSE_MAIN");
void* FLAME_LENGTH_SRB	= API.getParamHandle("FLAME_LENGTH_SRB");
void* FLAME_PULSE_SRB	= API.getParamHandle("FLAME_PULSE_SRB");

void* RADIO_OPACITY = API.getParamHandle("RADIO_OPACITY");

void* CMAKE_LAT = API.getParamHandle("CMAKE_LAT");
void* CMAKE_LON = API.getParamHandle("CMAKE_LON");
void* CMAKE_YAW = API.getParamHandle("CMAKE_YAW");

void* CMAKE_AA = API.getParamHandle("CMAKE_AA");
void* CMAKE_A = API.getParamHandle("CMAKE_A");
void* CMAKE_E = API.getParamHandle("CMAKE_E");
void* CMAKE_R = API.getParamHandle("CMAKE_R");
void* CMAKE_IAS = API.getParamHandle("CMAKE_IAS");

void* CMAKE_BROLL = API.getParamHandle("CMAKE_BROLL");
void* CMAKE_BPITCH = API.getParamHandle("CMAKE_BPITCH");

double CL_AoA_table[] = {
	-179.9,
	-32,
	-30,
	-26.5,
	-22.5,
	-21,
	-18.5,
	-16,
	-13.5,
	-11,
	0,
	11,
	13.5,
	16,
	18.5,
	21,
	22.5,
	26.5,
	30,
	32,
	179.9,

};
double CL_table[] = {
	0,
	0,
	-0.2,
	-0.8,
	-1.09,
	-1.13,
	-1.09,
	-0.99,
	-0.93,
	-0.75,
	0,
	0.75,
	0.93,
	0.99,
	1.09,
	1.13,
	1.09,
	0.8,
	0.2,
	0,
	0,

};
double CD0_AoA_table[] = {
	-180.0233392,
	-89.95437384,
	-14.89690267,
	0,
	14.89690267,
	89.95437384,
	180.0233392,
};
//double CD0_table[] = {
//	0.82,
//	0.39,
//	0.10,
//	0.064,
//	0.10,
//	0.39,
//	0.82,
//};
double CD0_table[] = {
	0.52,
	0.29,
	0.05,
	0.064,
	0.05,
	0.29,
	0.52,
};
double CDbeta_table[] = {
	0.006,
	0.1,
	0.04,
	0,0,
	0.04,
	0.1,
	0.006,
};


void add_local_force(const Vec3 & Force, const Vec3 & Force_pos)
{
	common_force.x += Force.x;
	common_force.y += Force.y;
	common_force.z += Force.z;

	/*Vec3 delta_pos(Force_pos.x - center_of_gravity.x,
				   Force_pos.y - center_of_gravity.y,
				   Force_pos.z - center_of_gravity.z);

	Vec3 delta_moment = cross(delta_pos, Force);

	common_moment.x += delta_moment.x;
	common_moment.y += delta_moment.y;
	common_moment.z += delta_moment.z;*/
}

void add_local_moment(const Vec3 & Moment)
{
	common_moment.x += Moment.x;
	common_moment.y += Moment.y;
	common_moment.z += Moment.z;
}

/*
set internal fuel volume , init function, called on object creation and for refueling ,
you should distribute it inside at different fuel tanks
*/
void   ed_fm_set_internal_fuel(double fuel)
{
	if (fuel >= 0 && fuel <= 72.7866)
	{
		centerTank = fuel;
	}
	else if (fuel > 72.7866)
	{
		centerTank = 72.7866;
	}
	else
	{
		centerTank = 0;
	}

	if (fuel > 72.7866)
	{
		wingTank = fuel - 72.7866;
	}
	else
	{
		wingTank = 0;
	}

	internal_fuel = fuel;
	cout << "ed_fm_set_internal_fuel: " << fuel << endl;
}
/*
get internal fuel volume
*/
double ed_fm_get_internal_fuel()
{
	//cout << "ed_fm_get_internal_fuel: " << internal_fuel << endl;
	return internal_fuel;
}
void simulate_fuel_consumption(double dt, double fuelFLow, double fuelSelect)
{
	if (Ignition)
	{
		internal_fuel -= 0.02;
		internal_fuel = limit(internal_fuel, 0, 1000);
	}
	//cout << "simulate_fuel_consumption: " << internal_fuel << endl;
	//New Unlimited Fuel
	//if (unlimited_fuel)
	//{
	//	return;
	//}
	
	//fuel_consumption_since_last_time = (6.01 * (fuelFLow / 3600) / 2.205) * dt;

	//if (fuel_consumption_since_last_time > internal_fuel)
	//	fuel_consumption_since_last_time = internal_fuel;

	//if (fuelSelect == .5)
	//{
	//	centerTank = limit(centerTank - fuel_consumption_since_last_time,0,9999);
	//	internal_fuel -= fuel_consumption_since_last_time;
	//	if (centerTank > 0)
	//	{
	//		isFeed = 1;
	//	}
	//	else
	//	{
	//		isFeed = 0;
	//	}
	//}
	//else if (fuelSelect == 1)
	//{
	//	wingTank = limit(wingTank - fuel_consumption_since_last_time, 0, 9999);
	//	internal_fuel -= fuel_consumption_since_last_time;
	//	if (wingTank > 0)
	//	{
	//		isFeed = 1;
	//	}
	//	else
	//	{
	//		isFeed = 0;
	//	}
	//}
	//else if (fuelSelect == 0)
	//{
	//	internal_fuel = internal_fuel;
	//	isFeed = 0;
	//}

}

void ed_fm_add_local_force(double & x,double &y,double &z,double & pos_x,double & pos_y,double & pos_z)
{
	x = common_force.x + throttle * 1000; // 1000 = 80kts
	y = common_force.y;
	z = common_force.z;
	pos_x = center_of_gravity.x;
	pos_y = center_of_gravity.y;
	pos_z = center_of_gravity.z;
}
void ed_fm_add_global_force(double & x,double &y,double &z,double & pos_x,double & pos_y,double & pos_z)
{

}
void ed_fm_add_global_moment(double & x,double &y,double &z)
{

}
void ed_fm_add_local_moment(double & x,double &y,double &z)
{
	x = common_moment.x;
	y = common_moment.y;
	z = common_moment.z;
}
double ed_fm_get_shake_amplitude()
{
	return shake;
}

void ed_fm_cold_start()
{
	//cout << "on mission start"<< endl;
	//if (!activity)
	//	activity = new ServiceActivity();

	
	ColdLaunchSystem();

	dome_light = 0;
	headlight = 0;
	hazzard_light = 0;
	Key_SW = 0;

	longStickInputArg = 0;
	latStickInputArg = 0;

	maniPress = 0;
	RPM = 0;
	RPM_Current = 0;
	cyl_fuel = 0;
	MagState = 0;
	FuelSelected = 0;
	MasterOnOff = 0;
	isFeed = 1;
	L_roll	  = 0;
	M_pitch	  = 0;
	N_yaw	  = 0;
	thrust = 0;
	trim_bias = 0;////edit this

	throttle = 0.0;
	latStickInput = 0.0;
	longStickInput = 0.0;
	pedalInput = 0.0;

	gear_pos	= 0;
	gear_target = 0;
}
void ed_fm_hot_start()
{

	trim_bias = 0.0;

	MASTER_SW = 1;
	FUEL_SW = 1;
	SRB_ARMED = 1;
	MFT_ARMED = 1;
	dome_light = 0;
	headlight = 0;
	hazzard_light = 0;
	Key_SW = 0;

	longStickInputArg = 0;
	latStickInputArg = 0;

	maniPress = 30;
	RPM = 700;
	RPM_Current = 700;
	cyl_fuel = 25;
	MagState = 0.75;
	FuelSelected = 0.5;
	MasterOnOff = 1;
	isFeed = 1;
	L_roll = 0;
	M_pitch = 0;
	N_yaw = 0;
	thrust = 0;


	throttle = 0.0;
	latStickInput = 0.0;
	longStickInput = 0.0;
	pedalInput = 0.0;
	
	gear_pos	= 0;
	gear_target = 0;
}
void ed_fm_hot_start_in_air()
{
	PilotHasControl = true;
	dome_light = 0;
	headlight = 0;
	hazzard_light = 0;
	Key_SW = 0;

	longStickInputArg = 0;
	latStickInputArg = 0;

	maniPress = 30;
	RPM = 1800;
	RPM_Current = 1800;
	cyl_fuel = 25;
	MagState = 0.75;
	FuelSelected = 0.5;
	MasterOnOff = 1;
	isFeed = 1;
	WoW = 0;
	L_roll = 0;
	M_pitch = 0;
	N_yaw = 0;
	thrust = 0;
	trim_bias = 0;

	throttle = 0.0;
	latStickInput = 0.0;
	longStickInput = 0.0;
	pedalInput = 0.0;

	gear_pos	= 0;
	gear_target = 0;
}

// Start of body simulation
void ed_fm_simulate(double dt)
{
	/*Instantiate API object
	EDAPI API;

	//Create a pointer to the Handle
	void * MAGSTATE_PARAM = API.getParamHandle("MAGSTATE_PARAM");*/

	// Pass in handle to the getParam method
	//MagState = API.getParam(MAGSTATE_PARAM);
	//cout << "CMAKE" << endl;

	//bool* show = new bool(true);
	//ImGui::ShowDemoWindow(show);
	LaunchDebug();

	radar_alt = API.getParam(RADALT);

	MixPos = API.getParam(MIXTURE_PARAM);
	double pitch_data = API.getParam(ADIPITCH);
	pitch_data = pitch_data * radiansToDegrees;

	API.setParam(SRB, SRBarg);
	API.setParam(ROCKET, RocketArg);
	API.setParam(SRB_JET, srb_jet);
	API.setParam(MAIN_JET, main_jet);
	API.setParam(SMOKE_SRB, SRBarg);
	API.setParam(SMOKE_MAIN, RocketArg);

	// Sim setup
	common_force  = Vec3();
	common_moment = Vec3();

	Vec3 airspeed;

	airspeed.x = velocity_world_cs.x - wind.x;
	airspeed.y = velocity_world_cs.y - wind.y;
	airspeed.z = velocity_world_cs.z - wind.z;

	V_propWind				= sqrt(airspeed.x * airspeed.x + airspeed.y * airspeed.y + airspeed.z * airspeed.z) * mpstofps; // Airspeed used to model dynamic pres from prop over controls
	V_indicated_airspeed	= sqrt(airspeed.x * airspeed.x + airspeed.y * airspeed.y + airspeed.z * airspeed.z) * mpstofps; // Airspeed used to indicated airspeed gauge

	if (V_propWind < 130) // Limit the airspeed to this number (airspeed over controls from prop) (so that at 0 airspeed there is still authority over controls)
	{
	V_propWind = 130;
	}

	normalizeSRB(internal_fuel);
	normalizeMFT(internal_fuel);

	updateGear();
	updatetheClock(dt);

	updateFlameLengthMain(dt);
	updateFlamePulseMain(dt);

	updateFlameLengthSRB (dt);
	updateFlamePulseSRB	 (dt);

	updateLaunchStatus();
	//updateRocketStage();
	//updateLaunchTrim(trim_bias,pitch_data);
	//updateBobbleRoll();
	updateBobblePitch();

	updateHeadlights(MASTER_SW, headlight);
	updateBrakelight(MASTER_SW, BrakeLeft);
	updateHazzard(MASTER_SW, hazzard_light, dt);
	updateDomelights(MASTER_SW, dome_light);

	updateHook(MASTER_SW, Hook_SW);
	
	//UpdateChutePitch();
	//updateDragChute(Drag_SW,indicated_airspeed);
	//updateChuteRoll();


	API.setParam(FLAME_LENGTH_MAIN, flame_length_M);
	API.setParam(FLAME_LENGTH_SRB, flame_length_R);
	API.setParam(FLAME_PULSE_MAIN, flame_pulse_M);
	API.setParam(FLAME_PULSE_SRB, flame_pulse_R);
	API.setParam(RADIO_OPACITY, MASTER_SW);



	/*if (theClock > 5)
	{
		trim_bias = 0.005;
	}*/

	//if (Ignition && theClock > 5)
	//{
	//	if (pitch_data < 88)
	//	{
	//		trim_bias += 0.0002;
	//		trim_bias = limit(trim_bias, -0.5, 0.5);
	//		cout << "TRIM BIAS-: " << trim_bias << endl;
	//	}
	//	else if (pitch_data > 88)
	//	{
	//		trim_bias -= 0.0002;
	//		trim_bias = limit(trim_bias, -0.5, 0.5);
	//		cout << "TRIM BIAS-: " << trim_bias << endl;
	//	}
	//	else if (pitch_data == 88)
	//	{
	//		trim_bias = trim_bias;
	//		trim_bias = limit(trim_bias, -0.5, 0.5);
	//		cout << "TRIM BIAS =: " << trim_bias << endl;
	//	}
	//	
	//}

	//cout << "RADALT: " << radar_alt << endl;

	// Engine
	//thrust                = throttle * thrust_aval * nuPropEff;
	updateRocket();
	engineFuelPump          = update_enginePump(RPM);
	ElectricBoostpsi        = update_electricPump(ElectricBoostOnOff);
		Mixture = MixPos;
	cyl_fuel	            = is_prime(engineFuelPump, ElectricBoostpsi, Mixture);
	
	RPM						= update_RPM(V_indicated_airspeed, throttle, PropPitch, fpstoknts, Mixture, fuelFlow_min, fuelFlow, engineFuelPump, ElectricBoostpsi, cyl_fuel, MagState, isFeed, maniPress);
	//RPM = update_RPM(V_indicated_airspeed, throttle, PropPitch, fpstoknts, Mixture, fuelFlow_min, fuelFlow, engineFuelPump, ElectricBoostpsi, cyl_fuel, MagState, isFeed);
	maniPress               = update_Mani(P, psftoinHg, throttle, RPM);
	maniPressGaugeArg       = update_Mani_gauge(maniPress);
	//API.setParam("MANIFOLD_PARAM", maniPress);
					                                                                        
	power                   = update_power(maniPress, P * psftoinHg, RPM, T, ElectricBoostpsi, engineFuelPump, Mixture) * 1.26;                                // Units in HP
	thrust                  = update_thrust(rho, power, prop_disc_area) * nuPropEff * nuEngineEff * limit(throttle, .1, 1);
					   
	fuelFlow_min            = update_fuelFlow_min(RPM, maniPress, power);
	fuelFlow		        = update_fuelFlow_actual(fuelFlow_min, Mixture, ElectricBoostpsi, RPM, MasterOnOff, isFeed);
	fuelFlowGaugeArg        = update_fuelFlow_gauge(fuelFlow);
	//API.setParam("FUELFLOW_PARAM", fuelFlow);

	simulate_fuel_consumption(dt, fuelFlow, FuelSelected);

	centerTankArg           = normalize_centerTank(centerTank, 72.7866, MasterOnOff, .02);
	wingTankArg	            = normalize_wingTank(wingTank, 86.4171, MasterOnOff, .02);
					        
	prop_animation	        = update_prop(RPM, MagState, MasterOnOff, Mixture);
	blur_animation			= update_blur(RPM);
	// Engine end
	
	V_freeStream	        = sqrt(airspeed.x * airspeed.x + airspeed.y * airspeed.y + airspeed.z * airspeed.z) * mpstofps + (thrust * (1 / V_propWind) * 3.0); // Calculate free stream velocity (freestream + induced airspeed from prop calculation)
	mach		            = V_freeStream / speed_of_sound;                                                                                            // Calculate mach (not used much in this code)

	if (V_freeStream < 0.01)                                                                                                                            // Limit free stream to be above 0 (must be .01 so that we are never deviding by zero - will crash your game)
	{
		V_freeStream = 0.01;
	}
	// End of sim setup

	Vec3 thrust_force(thrust_final, 0, 0);
	//Vec3 thrust_force_pos(-1.508, 0, 0);
	Vec3 thrust_force_pos(0, 0, 0);
	add_local_force(thrust_force, thrust_force_pos);

	// Aerodynamics
	q				        = .5 * rho * V_freeStream * V_freeStream * kgm3toslugft3;																	// Calculate free stream dynamic pressure
	q_airspeedGauge         = .5 * rho * V_indicated_airspeed * V_indicated_airspeed * kgm3toslugft3;                                                   // Dynamic pressure for airspeed gauge calculation
	q_forwardDir            = .5 * rho * simvx * simvx * kgm3toslugft3;

	alphadot	            = ((simvx * simay) - (simax * simvy)) / (V_indicated_airspeed * V_indicated_airspeed * cos(betaRad) * cos(betaRad));        // Calculate alpha dot
					        
	alpha_limit		        = limit(aoaRaw, -20, 26.5);                                                                                                   //20-20 limit aoa to this range
	aoaRad_limit			= limit(aoaRad, -1.309, 1.309);
					        
	CL				        = lerp(CL_AoA_table, CL_table, sizeof(CL_AoA_table) / sizeof(double), alpha_limit) * cos(abs(betaRad)) * cos(abs(betaRad)); // Calc Cl based on AoA from table
	L				        = calc_lift(CL, wingArea, q_forwardDir);                                                                                    // Calc lift
					        																					                                        
	CD0				        = lerp(CD0_AoA_table, CD0_table, sizeof(CD0_AoA_table) / sizeof(double), alpha_limit);                                           // Calc CD0 - profile drag based on AoA from table
	CDbeta			        = lerp(CD0_AoA_table, CDbeta_table, sizeof(CD0_AoA_table) / sizeof(double), betaRaw);                                       // Calc CDbeta based on beta from table
	CDde			        = calc_CDde(CDde, longStickInput, elevator_DEG, aoaRad, CL, q, wingArea);                                                   // Calc elevator drag
	CD				        = calc_CD(CD0, CDbeta, CDde, CDi);                                                                                          // Calc CD
	D				        = calc_drag(q, wingArea, CD);                                                                                               // Calc total drag
					        											                                                                                
	N				        = calc_normal(alpha_limit*degreestorads, L, D);                                                                                                // Convert to body axis normal force
	A			            = -calc_axial(alpha_limit*degreestorads, L, D);                                                                                                // Convert to body axis axial force
					        										                                                                                    
	Y				        = calc_side(wingArea, betaRad, q, CYb);                                                                                     // Calc side force
																	                                                                                    
																	                                                                                    
	Vec3 aero_force(A * lbtoN, N * lbtoN, Y * lbtoN);                                                                                                   // Add force (x,y,z)
	Vec3 aero_force_pos(0, 0, 0);                                                                                                                       // Add postiion (Aerodynamic reference center)
	add_local_force(aero_force, aero_force_pos);                                                                                                        // Add force

	// Calc moments
	L_roll			        = rollmoment(Clb, Cldadot, Clda/6, Cldr, q, q_airspeedGauge, meanChord, wingArea, latStickInput, aileron_DEG, rudder_DEG, wingSpan, betaRad, rollRate_rps, rho, u, (limit(aoaRaw,-179,16) * (M_PI / 180)), Clr, V_indicated_airspeed, pedalInput, yawRate_rps, angle_of_roll, V_indicated_airspeed, longStickInput, aoaRad_limit*radiansToDegrees, (betaRad * (180 / M_PI)));
	M_pitch			        = pitchmoment(Cmalpha, Cmde, Cmq * (1.9 * abs(rollRate_rps)), Cmadot, q , q_airspeedGauge, q_forwardDir, meanChord, wingArea, longStickInput, elevator_DEG, rudder_DEG, wingSpan, betaRad, pitchRate_rps, rho, u, aoaRad_limit, V_indicated_airspeed, pedalInput, alphadot, trim_bias, WoW);
	N_yaw			        = yawmoment(Cnb, Cnr, Cndr, Cnda, q, q_forwardDir, meanChord, wingArea, latStickInput, aileron_DEG, rudder_DEG, wingSpan, betaRad, yawRate_rps, rho, u, aoaRad_limit, V_indicated_airspeed, pedalInput);

	J                       = calc_advance_ratio(u, (RPM / 60), Di);

	Q_tourque = 0;//(RPM / 130) * rho * kgm3toslugft3 * pow((RPM / 60), 2) * pow(Di,5) * pow(limit(V_indicated_airspeed / 260, .04, 1),2);

	Vec3 Moment_total_L((L_roll + -Q_tourque) * ftlbtonm, 0, 0);
	add_local_moment(Moment_total_L);

	Vec3 Moment_total_N(0, N_yaw * ftlbtonm, 0);
	add_local_moment(Moment_total_N);

	Vec3 Moment_total_M(0, 0, M_pitch * ftlbtonm);
	add_local_moment(Moment_total_M);

	// Cockpit and Model
	P0				        = calc_total_pressure(q_airspeedGauge, P);
	indicated_airspeed      = calc_indicated_airspeed(V_indicated_airspeed, kgm3toslugft3, P0, P) * fpstoknts;
	RPM_light_state         = update_RPM_lights(RPM);
					        
	string_x				= string_x_motion(simvx, windvx, V_indicated_airspeed);
	string_y				= string_y_motion(simvy, windvy, V_indicated_airspeed);
	string_z				= string_z_motion(simvz, windvz);

	if (API.getParam(SPRING_KEY_PARAM) >= 0.9)
	{
		MagState = 0.75;
	}

	//shake                   = (aoaRaw + ((simax + simay + simaz - 87.4)/23)) / 55 * limit(1 - WoW, 0, 1); // 55 * limit(1 - WoW, 0, 1);
	//New Shake Statement
	if (RPM > 500)
	{
		shake = (aoaRaw + ((simax + simay + simaz - 87.4) / 23)) / 55 * limit(1 - WoW, 0, 1); // 55 * limit(1 - WoW, 0, 1);
	}
	else
	{
		shake = 0;
	}

	////Gear Logic
	//if (gear_target == 1 && gear_pos < 0.98)
	//{
	//	gear_pos += 0.0002;
	//	gear_pos = limit(gear_pos, 0, 0.98);
	//	std::cout << "LAUNCH POSITION: " << gear_pos << std::endl;
	//}
	
	updateCountdown(dt);
	//cout << indicated_airspeed / 230 << endl;
	//else if (gear_target == 0 && gear_pos > 0)
	//{
	//	gear_pos = gear_pos - dt / 4;
	//	std::cout << "GEAR POSITION: " << gear_pos << std::endl;
	//}

	//std::cout << "Mixture: " << MixPos << std::endl;
	//std::cout << "Shake: " << shake << std::endl;
	//std::cout << "RAW RPM: " << RPM << std::endl;
	//std::cout << indicated_airspeed/240 << std::endl;

	//file i/o logic
	/*ofstream fwrite;
	fwrite.open("C:\\Users\\Brandon\\Saved Games\\DCS.openbeta\\Mods\\aircraft\\Edge 540\\FF.txt");

	//fwrite << "SightPos: " + to_string(sightPosition) + "\n" + "LeftOff: " + to_string(LeftSightOff) + "\n" + "RightOff: " + to_string(RightSightOff);// +"\n" + "Feed: " + to_string(cyl_fuel) + "\n" + "FUEL SELECT: " + to_string(FuelSelected) + "\n" + "PRIMED: " + to_string(isprimed);
	fwrite << to_string(Q_tourque);

	fwrite.flush();
	fwrite.close();*/


	//API.setParam(CMAKE_LAT, latStickInputArg);
	//API.setParam(CMAKE_LON, -longStickInputArg);
	//API.setParam(CMAKE_YAW, pedalInputArg);

	//API.setParam(CMAKE_AA, -latStickInput);
	//API.setParam(CMAKE_A, latStickInput);
	//API.setParam(CMAKE_E, longStickInput);
	//API.setParam(CMAKE_R, -pedalInput);
	//API.setParam(CMAKE_IAS, indicated_airspeed/230);
	//API.setParam(CMAKE_BROLL, bobble_roll);
	//API.setParam(CMAKE_BPITCH, bobble_pitch);


}

void ed_fm_set_atmosphere(double h,//altitude above sea level
							double t,//current atmosphere temperature , Kelwins
							double a,//speed of sound
							double ro,// atmosphere density
							double p,// atmosphere pressure
							double wind_vx,//components of velocity vector, including turbulence in world coordinate system
							double wind_vy,//components of velocity vector, including turbulence in world coordinate system
							double wind_vz //components of velocity vector, including turbulence in world coordinate system
						)
{
	wind.x = wind_vx;
	wind.y = wind_vy;
	wind.z = wind_vz;

	rho				   = ro;
	speed_of_sound     = a;
	P				   = p * Nm2topsf;
	T                  = t - 273.15;
}
/*
called before simulation to set up your environment for the next step
*/
void ed_fm_set_current_mass_state (double mass,
									double center_of_mass_x,
									double center_of_mass_y,
									double center_of_mass_z,
									double moment_of_inertia_x,
									double moment_of_inertia_y,
									double moment_of_inertia_z
									)
{
	center_of_gravity.x  = center_of_mass_x;
	center_of_gravity.y  = center_of_mass_y;
	center_of_gravity.z  = center_of_mass_z;
}

/*
called before simulation to set up your environment for the next step
*/
void ed_fm_set_current_state (double ax,//linear acceleration component in world coordinate system
							double ay,//linear acceleration component in world coordinate system
							double az,//linear acceleration component in world coordinate system
							double vx,//linear velocity component in world coordinate system
							double vy,//linear velocity component in world coordinate system
							double vz,//linear velocity component in world coordinate system
							double px,//center of the body position in world coordinate system
							double py,//center of the body position in world coordinate system
							double pz,//center of the body position in world coordinate system
							double omegadotx,//angular accelearation components in world coordinate system
							double omegadoty,//angular accelearation components in world coordinate system
							double omegadotz,//angular accelearation components in world coordinate system
							double omegax,//angular velocity components in world coordinate system
							double omegay,//angular velocity components in world coordinate system
							double omegaz,//angular velocity components in world coordinate system
							double quaternion_x,//orientation quaternion components in world coordinate system
							double quaternion_y,//orientation quaternion components in world coordinate system
							double quaternion_z,//orientation quaternion components in world coordinate system
							double quaternion_w //orientation quaternion components in world coordinate system
							)
{
	velocity_world_cs.x = vx;
	velocity_world_cs.y = vy;
	velocity_world_cs.z = vz;
}


void ed_fm_set_current_state_body_axis(double ax,//linear acceleration component in body coordinate system
	double ay,//linear acceleration component in body coordinate system
	double az,//linear acceleration component in body coordinate system
	double vx,//linear velocity component in body coordinate system
	double vy,//linear velocity component in body coordinate system
	double vz,//linear velocity component in body coordinate system
	double wind_vx,//wind linear velocity component in body coordinate system
	double wind_vy,//wind linear velocity component in body coordinate system
	double wind_vz,//wind linear velocity component in body coordinate system

	double omegadotx,//angular accelearation components in body coordinate system
	double omegadoty,//angular accelearation components in body coordinate system
	double omegadotz,//angular accelearation components in body coordinate system
	double omegax,//angular velocity components in body coordinate system
	double omegay,//angular velocity components in body coordinate system
	double omegaz,//angular velocity components in body coordinate system
	double yaw,  //radians
	double pitch,//radians
	double roll, //radians
	double common_angle_of_attack, //AoA radians
	double common_angle_of_slide   //AoS radians
	)
{
	simvy			= vy * mpstofps;
	simvz			= vz * mpstofps;
	simvx			= vx * mpstofps;
	simax           = ax * mps2tofps2;
	simay           = ay * mps2tofps2;
	simaz           = az * mps2tofps2;
	windvx		    = wind_vx * mpstofps;
	windvy			= wind_vy * mpstofps;
	windvz			= wind_vz * mpstofps;
	aoaRad			= common_angle_of_attack;
	aoaRaw			= common_angle_of_attack * radiansToDegrees;
	betaRaw			= common_angle_of_slide * radiansToDegrees;
	betaRad		    = common_angle_of_slide;
	rollRate_rps	= omegax;
	pitchRate_rps	= omegaz;
	yawRate_rps		= -omegay;
	u				= vx * mpstofps;
	angle_of_roll   = roll * radiansToDegrees;
}
/*
input handling
*/
void ed_fm_set_command (int command, float value)
{	
	//if (command == inputs::MagSelect)
	//{
	//	MagState = limit(((int)((8 * value - 26) * 4) * .25f), 0, 1);
	//	cout << "MAGS: " << MagState << endl;
	//	cout << "MAGS VALUE: " << value << endl;
	//}

	//if (command == inputs::FuelSelect)
	//{
	//	FuelSelected = limit(8 * value - 26, 0, 1);
	//}

	if (value > 1) // if the command comes from clickabledata.lua, it adds the device id to the value and changes the value
	{
		float device_id;
		float normalized = modf(value, &device_id);
		value = normalized * 8.f - 2.f;
	}
	switch (command)
	{
	case inputs::JoystickThrottle:
		if (Key_SW && radar_alt < 0.8)
		{
			throttle = 0.5 * (-value + 1.0);
			cout << "THROTTLE: " << throttle << endl;
		}
		throttle = 0.5 * (-value + 1.0);
		break;
	case inputs::JoystickPitch:
		longStickInputArg = limit(value, -1.0, 1.0);
		if (PilotHasControl)
		{
			longStickInput = limit(value, -1.0, 1.0);
		}
		break;
	case inputs::JoystickRoll:
		latStickInputArg = limit(value, -1.0, 1.0);
		//cout << "ROLL: " << latStickInputArg << endl;
		if (PilotHasControl)
		{
			latStickInput = limit(value, -1.0, 1.0);
		}
		break;
	case inputs::JoystickYaw:
		pedalInputArg = limit(value, -1.0, 1.0);
		if (PilotHasControl)
		{
			pedalInput = limit(value, -1.0, 1.0);
		}
		break;
	case inputs::TrimUp:
		//trim_bias = limit(trim_bias + 0.0002, -0.5, 0.5);
		//cout << "TRIM: " << trim_bias << endl;
		if (PilotHasControl)
		{
			trim_bias = limit(trim_bias + 0.0002, -0.5, 0.5);
			cout << "TRIM: " << trim_bias << endl;
		}
		break;
	case inputs::TrimDown:
		//trim_bias = limit(trim_bias - 0.0002, -0.5, 0.5);
		//cout << "TRIM: " << trim_bias << endl;
		if (PilotHasControl)
		{
			trim_bias = limit(trim_bias - 0.0002, -0.5, 0.5);
			cout << "TRIM: " << trim_bias << endl;
		}
		break;
	case inputs::TrimNeutral:
		if (PilotHasControl)
		{
			trim_bias = 0.0;
			cout << "TRIM: " << trim_bias << endl;
		}
		break;
	case inputs::WheelBrake:
		BrakeLeft = 0.5 * (-value + 1.0);
		BrakeRight = 0.5 * (-value + 1.0);
		cout << "BRAKES LEFT: " << BrakeLeft << " BRAKES RIGHT: " << BrakeRight << endl;
		break;
	case inputs::LaunchPrep:
		if (MASTER_SW)
		{
			cout << "LAUNCH PREP: " << gear_target << endl;
			if (gear_target == 0)
			{
				gear_target = 1;
				cout << "GEAR TARGET: " << gear_target << endl;
			}
		}
		//else if (gear_target == 1)
		//{
		//	gear_target = 0;
		//	cout << "GEAR TARGET: " << gear_target << endl;
		//}
		break;


	case inputs::Master:
		MASTER_SW = limit(value, 0, 1);
		cout << "MASTER: " << MASTER_SW << endl;
		break;
	case inputs::FuelPump:
		FUEL_SW = limit(value, 0, 1);
		cout << "FUEL: " << FUEL_SW << endl;
		break;
	case inputs::SRB_ARM:
		SRB_ARMED = limit(value, 0, 1);
		cout << "SRB ARMED: " << SRB_ARMED << endl;
		break;
	case inputs::MFT_ARM:
		MFT_ARMED = limit(value, 0, 1);
		cout << "MFT ARMED: " << MFT_ARMED << endl;
		break;

	case inputs::Lights:
		headlight = limit(value, 0, 1);
		cout << "HEADLIGHT: " << headlight << endl;
		break;
	case inputs::Dome:
		dome_light = limit(value, 0, 1);
		cout << "DOME: " << dome_light << endl;
		break;
	case inputs::Hazzard:
		hazzard_light = limit(value, 0, 1);
		cout << "HAZZARD: " << hazzard_light << endl;
		break;

	case inputs::Key:
		Key_SW = limit(value, 0, 1);
		cout << "KEY: " << Key_SW << endl;
		break;

	case inputs::Hook:
		Hook_SW = limit(value, 0, 1);
		cout << "HOOK: " << Hook_SW << endl;
		break;
	case inputs::Drag:
		Drag_SW = limit(value, 0, 1);
		cout << "CHUTE: " << Drag_SW << endl;
		break;
	case inputs::Countdown:
		if (LaunchReady && gear_pos == 0.98)
		{
			LaunchCountdown = true;
			cout << "LAUNCH" << endl;
		}
		break;

	}
}						

/*
	Mass handling 

	will be called  after ed_fm_simulate :
	you should collect mass changes in ed_fm_simulate 

	double delta_mass = 0;
	double x = 0;
	double y = 0; 
	double z = 0;
	double piece_of_mass_MOI_x = 0;
	double piece_of_mass_MOI_y = 0; 
	double piece_of_mass_MOI_z = 0;
 
	//
	while (ed_fm_change_mass(delta_mass,x,y,z,piece_of_mass_MOI_x,piece_of_mass_MOI_y,piece_of_mass_MOI_z))
	{
	//internal DCS calculations for changing mass, center of gravity,  and moments of inertia
	}
*/
bool ed_fm_change_mass  (double & delta_mass,
						double & delta_mass_pos_x,
						double & delta_mass_pos_y,
						double & delta_mass_pos_z,
						double & delta_mass_moment_of_inertia_x,
						double & delta_mass_moment_of_inertia_y,
						double & delta_mass_moment_of_inertia_z
						)
{
	if (fuel_consumption_since_last_time > 0)
	{
		delta_mass		 = fuel_consumption_since_last_time;
		delta_mass_pos_x = 0; // -1.0
		delta_mass_pos_y = 0; // 1.0
		delta_mass_pos_z = 0;

		delta_mass_moment_of_inertia_x = 0;// wingTank / 120;
		delta_mass_moment_of_inertia_y	= 0;
		delta_mass_moment_of_inertia_z	= 0;

		fuel_consumption_since_last_time = 0; // set it 0 to avoid infinite loop, because it called in cycle 
		// better to use stack like structure for mass changing 
		return true;
	}
	else 
	{
		return false;
	}
}

/*
	set external fuel volume for each payload station , called for weapon init and on reload
*/
void  ed_fm_set_external_fuel (int	 station,
								double fuel,
								double x,
								double y,
								double z)
{

}
/*
	get external fuel volume 
*/
double ed_fm_get_external_fuel ()
{
	return 0;
}
//ed_fm_set_draw_args_v2(float* array, size_t size);
void ed_fm_set_draw_args_v2(float* array, size_t size)//ed_fm_set_draw_args (EdDrawArgument * drawargs,size_t size)
{

	//data[27] = (float)(throttle);
	//data[28] = (float)(throttle);
	//data[29] = (float)(throttle);
	//data[30] = (float)(throttle);
	
	//drawargs[1].f = (float)(1); // was * 3

	array[11]  = (float)(-latStickInput); // was * 3
	array[12]  = (float)(latStickInput); // was * 3
	array[13]  = (float)(longStickInput); // was * 3
	array[14]  = (float)(-pedalInput); // was * 3

	array[25] = (float)(hook_pos);//HOOK

	array[28]  = (float)(SRBarg);//for flame and sound
	array[29]  = (float)(RocketArg);//for flame and sound

	//array[35] = (float)(deploy_pos);  //force set closed 
	//array[36] = (float)(chute_pitch);  //force set closed 
	//array[37] = (float)(chute_roll);  //force set closed 

	array[38]  = (float)(0);  //force set closed 

	array[400] = (float)(0);  //force set due to HAB baloon rotation for flame
	array[404] = (float)(flame_length_M);
	array[406] = (float)(flame_pulse_M);

	array[405] = (float)(flame_length_R);
	array[407] = (float)(flame_pulse_R);

	array[415] = (float)(headlight_state);
	array[416] = (float)(hazzard_state);
	array[417] = (float)(brakeLight);


	//drawargs[15].f	 = (float)(trim_bias); // was * 10
	//drawargs[20].f   = (float)(prop_animation);

	//NO SIGHTS ON EDGE 540 save for 330?
	//drawargs[25].f   = (float)(sightPosition * 3.3 + RightSightOff  * 4);
	//drawargs[26].f   = (float)(sightPosition * 3.3 + LeftSightOff * 4);
	
	//drawargs[88].f = (float)(.66);//Integrity Check

	//drawargs[45].f = (float)(blur_animation);//Moved from lua to c++

	//WoW              = drawargs[4].f + drawargs[6].f;

	//if (size > 616)
	//{	
	//	drawargs[611].f = drawargs[0].f;
	//	drawargs[614].f = drawargs[3].f;
	//	drawargs[616].f = drawargs[5].f;
	//}

}
//Updated to the new 2.7 _v2 drw arg style.
void ed_fm_set_fc3_cockpit_draw_args_v2 (float* array, size_t size)
{

	array[1]		= (float)(-longStickInputArg);
	array[2]		= (float)(latStickInputArg);
	array[3]		= (float)(throttle);
	array[20]		= (float)(pedalInputArg);
	array[5]		= (float)(indicated_airspeed/230); //(indicated_airspeed/260)

	array[68]		= (float)(bobble_pitch);
	array[69]		= (float)(bobble_roll);

	array[81]		= (float)(srb_fuel);
	array[82]		= (float)(main_fuel);
	array[83]		= (float)(srb_fuel);

	array[91]		= (float)(domelight_state);

	//data[10] 	= (float)(-trim_bias * 2.9733 + (1-MasterOnOff));
	//data[12]    = (float)(PropPitch);
	//data[13]    = (float)(maniPressGaugeArg);
	//data[15]    = (float)(fuelFlowGaugeArg);
	//data[17]	= (float)(Mixture); //this is a failure atm due to not being able top link the mouse click to the dll command?
	//data[16]    = (float)(RPM_light_state);
	//data[22]    = (float)(centerTankArg);
	//data[23]    = (float)(wingTankArg);

}

void ed_fm_configure(const char * cfg_path)
{

}

double test_gear_state = 0;
double ed_fm_get_param(unsigned index)
{
	if (index <= ED_FM_END_ENGINE_BLOCK)
	{
		switch (index)
		{
		//case ED_FM_ENGINE_0_RPM:
		//case ED_FM_ENGINE_0_RELATED_RPM:	
		//case ED_FM_ENGINE_0_THRUST:			
		//case ED_FM_ENGINE_0_RELATED_THRUST:	
		case ED_FM_ENGINE_1_RPM:
			
		case ED_FM_ENGINE_1_RELATED_RPM:

		case ED_FM_ENGINE_1_THRUST:

		case ED_FM_ENGINE_1_RELATED_THRUST:

		case ED_FM_ENGINE_1_CORE_THRUST:

		case ED_FM_ENGINE_1_CORE_RELATED_THRUST:
			//return 0;
		case ED_FM_ENGINE_1_CORE_RPM:
			return 0;// throttle;
		case ED_FM_ENGINE_1_CORE_RELATED_RPM:
			return 0;// 0;// RPM / 2750;
		case ED_FM_ENGINE_2_RPM:
		case ED_FM_ENGINE_2_RELATED_RPM:
		case ED_FM_ENGINE_2_THRUST:
		case ED_FM_ENGINE_2_RELATED_THRUST:
		case ED_FM_ENGINE_2_CORE_THRUST:
		case ED_FM_ENGINE_2_CORE_RELATED_THRUST:
		case ED_FM_ENGINE_2_CORE_RPM:
		case ED_FM_ENGINE_2_CORE_RELATED_RPM:
			return 0;// RPM / 2750;
		//case ED_FM_ENGINE_3_RPM:
		//case ED_FM_ENGINE_4_RPM:
		//case ED_FM_ENGINE_5_RPM:

			//return 0;// RPM / 2750;

		case ED_FM_PROPELLER_0_RPM:					// propeller RPM , for helicopter it is main rotor RPM
			return 0;// throttle * 480;
		//case ED_FM_PROPELLER_1_PITCH:				// propeller blade pitch
		//case ED_FM_PROPELLER_1_TILT:				// for helicopter
		//case ED_FM_PROPELLER_1_INTEGRITY_FACTOR:   // for 0 to 1 , 0 is fully broken ,
			//return 1;
		//case ED_FM_PISTON_ENGINE_1_MANIFOLD_PRESSURE:
			//return maniPress * 3386.39;
		default:
			break;
		}
	}
	//else if (index >= ED_FM_SUSPENSION_0_RELATIVE_BRAKE_MOMENT &&
	//		 index < ED_FM_OXYGEN_SUPPLY)
	//{
	//	static const int block_size = ED_FM_SUSPENSION_1_RELATIVE_BRAKE_MOMENT - ED_FM_SUSPENSION_0_RELATIVE_BRAKE_MOMENT;
	//	switch (index)
	//	{
	//	case 0 * block_size + ED_FM_SUSPENSION_0_GEAR_POST_STATE:
	//	case 1 * block_size + ED_FM_SUSPENSION_0_GEAR_POST_STATE:
	//	case 2 * block_size + ED_FM_SUSPENSION_0_GEAR_POST_STATE:
	//		return test_gear_state;
	//	}
	//}

	switch (index)
	{
	case ED_FM_SUSPENSION_0_GEAR_POST_STATE:
	case ED_FM_SUSPENSION_0_DOWN_LOCK:
	case ED_FM_SUSPENSION_0_UP_LOCK:
		return 1.0;
	case ED_FM_SUSPENSION_1_GEAR_POST_STATE:
	case ED_FM_SUSPENSION_1_DOWN_LOCK:
	case ED_FM_SUSPENSION_1_UP_LOCK:
		return 1.0;
	case ED_FM_SUSPENSION_2_GEAR_POST_STATE:
	case ED_FM_SUSPENSION_2_DOWN_LOCK:
	case ED_FM_SUSPENSION_2_UP_LOCK:
		return 1.0;
	case ED_FM_SUSPENSION_0_WHEEL_YAW:
		return ((-pedalInputArg) * 50 * M_PI / 180); //made neg num to fix backwards steering
	case ED_FM_SUSPENSION_1_RELATIVE_BRAKE_MOMENT:
		return (BrakeLeft);
	case ED_FM_SUSPENSION_2_RELATIVE_BRAKE_MOMENT:
		return (BrakeRight);
	case ED_FM_OXYGEN_SUPPLY:
		return 101000;// O2_Pressure
	case ED_FM_FLOW_VELOCITY:
		return 1.0;
	case ED_FM_ANTI_SKID_ENABLE:
		return 1;
	}

	return 0;

}


bool ed_fm_add_local_force_component( double & x,double &y,double &z,double & pos_x,double & pos_y,double & pos_z )
{
	return false;
}
bool ed_fm_add_global_force_component( double & x,double &y,double &z,double & pos_x,double & pos_y,double & pos_z )
{
	return false;
}
bool ed_fm_add_local_moment_component( double & x,double &y,double &z )
{
	return false;
}
bool ed_fm_add_global_moment_component( double & x,double &y,double &z )
{
	return false;
}

bool ed_fm_enable_debug_info()
{
	return false;
}
void ed_fm_unlimited_fuel(bool value)
{
	unlimited_fuel = value;
	//std:cout << "UNLIMITED FUEL FUNCTION: " << value << std::endl;
}

//SDK might not be used
void ed_on_mission_start()
{
	
};
void ed_fm_set_plugin_data_install_path(const char* installpath)
{

}
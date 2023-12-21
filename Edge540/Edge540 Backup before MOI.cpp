// ED_FM_Template.cpp : Defines the exported functions for the DLL application.
#include "stdafx.h"
#include "Edge540.h"
#include "ED_FM_Utility.h"
#include <Math.h>
#include <stdio.h>
#include <string>

// Model Headers
#include "Controls/Controls.h"
#include "Aerodynamics/Aerodynamics.h"
#include "Engine/Engine.h"
#include "Cockpit/Cockpit.h"

Vec3	common_moment;
Vec3	common_force;
Vec3    center_of_gravity;
Vec3	wind;
Vec3	velocity_world_cs;


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

// Atmoshphere
double		V_freeStream			 = 0.0;				// Free stream velocity
double		V_propWind				 = 0.0;				// Free stream velocity
double		rho						 = 0.0;				// Desnity
double		q						 = 0.0;				// Dynamic Pressure

// Body
double		simvy					 = 0.0;
double		simvz					 = 0.0;
double		simvx					 = 0.0;
double      simax					 = 0.0;
double      simay					 = 0.0;
double      simaz					 = 0.0;
double      aoaRaw					 = 0.0;				// Raw AoA
double		aoaRad					 = 0.0;
double		betaRaw					 = 0.0;				// Raw beta
double		betaRad					 = 0.0;				// Raw beta


double		inertia_Ix_KGM2			 = 12874.0;			// Reference moment of inertia (kg/m^2)
double		inertia_Iy_KGM2			 = 75673.6;			// Reference moment of inertia (kg/m^2)
double		inertia_Iz_KGM2			 = 85552.1;			// Reference moment of inertia (kg/m^2)
double		alpha_limit				 = 0.0;				// Angle of attack (deg)
double      beta_limit				 = 0.0;				// Slideslip angle (deg)
double		rollRate_rps			 = 0.0;				// Body roll rate (rad/sec)
double		pitchRate_rps		     = 0.0;				// Body pitch rate (rad/sec)
double		yawRate_rps				 = 0.0;				// Body yaw rate (rad/sec)
double		u						 = 0.0;				// Forward Velocity
double		u_freestream			 = 0.0;

// Lifting Surfaces
double		wingSpan				= 24.3;				// Extra wing-span (m)
double		wingArea				= 98.00;			// Extra wing area (m^2)
double		meanChord				= 4.03;				// Extra mean aerodynamic chord (m)

double		elevator_DEG			= 25;				// Elevator deflection (deg)
double		rudder_DEG			    = 30;				// Rudder  deflection (deg)
double		aileron_DEG	            = 30;				// Aileron deflection (deg)
double		trim_bias				= 0.0;

// Aero forces
double		N						= 0.0;				// Normal force lift
double		A						= 0.0;				// Axial force
double		L						= 0.0;				// Body axis lift force
double		Y					    = 0.0;				//Side force
double		CL						= 0.0;				// Lift coeff

double	    thrust					= 0.0;
double	    thrust_aval				= 6000;

double		D						= 0.0;				// Body axis drag force
double		CD						= 0.0;				// Drag coeff
double		CD0						= 0.0;				// Profile Drag coeff
double		CDi						= 0.07;			// Induced Drag coeff (Function of CL)
double		CDde					= 0.05;			    // Drag due to Elevator Deflection
double		CDbeta					= 0.0;			    // Drag due to beta

double		CYb						= -2.2;				// Side force due to beta

double		AR = (wingSpan * wingSpan) / wingArea;		// Aspect Ratio

double		Cm						= 0.0;				// Pitch moment
double		Cmalpha					= -1.2;
double		Cmde					= 1.1;				// Pitch moment due to elevator
double		Cmq						= -1.466;			// Pitch moment due to pitch rate extended to work at zero velocity
double		Cmadot					= -7;				// Pitch moment due to alpha rate
double		alphadot				= 0.0;				// Time rate of change for alpha

double		Clda					= 0.28;		        // Roll moment due to ailerons
double		Cldr					= 0.0035;		    // Roll moment due to ailerons
double		Cldadot					= -.4;				// Roll moment due to roll rate
double		Clr						= 0.15;             // Roll moment due to yaw rate
double	    Clb						= -0.1;             // Roll moment due to beta

double		Cn						= 0.0;				// Yaw moment
double		Cnb						= -0.12;            // Yaw moment due to beta
double		Cnr						= 0.1411;			// Yaw moment due to yaw rate extended to work at zero velocity
double		Cndr					= -0.035;			// Yaw moment due to rudder
double		Cnda				    = -0.01;			// Adverse Yaw

double		M_pitch					= 0.0;
double		L_roll					= 0.0;
double		N_yaw					= 0.0;

// Engine
double cubicInchDisplacement		= 541.5;			// Cubic ft displacement

double engineBTU					= 5951700;			// Model is constant for now -> Make variable by linerally
double horsePower					= 0.0;

double fuelMixture				    = 0.0;				// Mixture is from 0-1		(1/9 rich = .111) (1/18 lean = .055) -> (2/14.7 rich = .14) (.5/14.7 -> .034 idle cutoff)
double airMixture					= 14.7;				// How much air the carb is pulling in -- fuelMixture / airMixture -- 1/15 > .5/15 >> 14.7 is chemically correct
double fueltoair					= 0.0;
double nuPropEff				    = 0.85;				// Double check this
double manifoldPress				= 0.0;

double PropPitch					= 0.0;
double RPM							= 0.0;

// Cockpit
double P0							= 0.0;				// Total Pressure
double P							= 0.0;				// Static Pressure
double q_airspeedGauge				= 0.0;
double indicated_airspeed		    = 0.0;
double V_indicated_airspeed			= 0.0;

double CL_AoA_table[] = {
	-179.9087477,
	- 135.2180397,
	- 89.95437384,
	- 44.69070802,
	- 34.37746771,
	- 16.04281826,
	- 14.32394488,
	- 12.60507149,
	- 11.4591559,
	0,
	11.4591559,
	12.60507149,
	14.32394488,
	16.04281826,
	34.37746771,
	44.69070802,
	89.95437384,
	135.2180397,
	179.9087477,
};

double CL_table[] = {
	0.05,
	1.0,
	0,
	0,
	1.2,
	-1.32,
	-1.45,
	-1.44,
	-1.4,
	0,
	1.4,
	1.44,
	1.45,
	1.32,
	1.2,
	0,
	0.0,
	-1.0,
	0.05,
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

double CD0_table[] = {
	0.06,
	0.6,
	0.058,
	0.049,
	0.058,
	0.06,
	0.06,
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

void simulate_fuel_consumption(double dt)
{
	fuel_consumption_since_last_time =  1 * throttle * dt; //1 kg persecond
	if (fuel_consumption_since_last_time > internal_fuel)
		fuel_consumption_since_last_time = internal_fuel;
	internal_fuel -= fuel_consumption_since_last_time;
}

void ed_fm_add_local_force(double & x,double &y,double &z,double & pos_x,double & pos_y,double & pos_z)
{
	x = common_force.x;
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

// Start of body simulation
void ed_fm_simulate(double dt)
{

	// Sim setup
	common_force  = Vec3();
	common_moment = Vec3();

	Vec3 airspeed;

	airspeed.x = velocity_world_cs.x - wind.x;
	airspeed.y = velocity_world_cs.y - wind.y;
	airspeed.z = velocity_world_cs.z - wind.z;

	V_propWind				= sqrt(airspeed.x * airspeed.x + airspeed.y * airspeed.y + airspeed.z * airspeed.z) * mpstofps;
	V_indicated_airspeed	= sqrt(airspeed.x * airspeed.x + airspeed.y * airspeed.y + airspeed.z * airspeed.z) * mpstofps;

	q_airspeedGauge = .5 * rho * V_indicated_airspeed * V_indicated_airspeed * kgm3toslugft3;

	if (V_propWind < 95)
	{
	V_propWind = 95;
	}

	V_freeStream	= sqrt(airspeed.x * airspeed.x + airspeed.y * airspeed.y + airspeed.z * airspeed.z) * mpstofps + (thrust * (1 / V_propWind) * 1.0);
	mach		    = V_freeStream / speed_of_sound;

	if (V_freeStream < 0.01)
	{
		V_freeStream = 0.01;
	}
	// End of sim setup

	// Engine


	fueltoair		= FtoA(fuelMixture, airMixture);
	horsePower		= HP(engineBTU, fueltoair, rho, nuPropEff, cubicInchDisplacement, RPM);

	//thrust			= horsePower / V_freeStream;
	thrust		    = throttle * thrust_aval;

	Vec3 thrust_force(thrust, 0, 0);
	Vec3 thrust_force_pos(0, 0, 0);
	add_local_force(thrust_force, thrust_force_pos);

	Vec3 Moment_thrust(0, 0, 0);
	add_local_moment(Moment_thrust);

	// Aerodynamics
	q				= .5 * rho * V_freeStream * V_freeStream * kgm3toslugft3;
	alphadot	    = ((simvx * simay) - (simax * simvy)) / (V_freeStream * V_freeStream * cos(betaRad) * cos(betaRad));

	alpha_limit		= limit(aoaRaw, -20, 20); //20-20

	CL				= lerp(CL_AoA_table, CL_table, sizeof(CL_AoA_table) / sizeof(double), alpha_limit);
	L				= calc_lift(CL, wingArea, q);

	CD0				= lerp(CD0_AoA_table, CD0_table, sizeof(CD0_AoA_table) / sizeof(double), aoaRaw);
	CDbeta			= lerp(CD0_AoA_table, CDbeta_table, sizeof(CD0_AoA_table) / sizeof(double), betaRaw);
	CDde			= calc_CDde(CDde, longStickInput, elevator_DEG, aoaRad, CL, q, wingArea);
	CD				= calc_CD(CD0, CDbeta, CDde, CDi);
	D				= calc_drag(q, wingArea, CD);

	N				= calc_normal(aoaRad, L, D);
	A			    = -calc_axial(aoaRad, L, D);

	Y				= calc_side(wingArea, betaRad, q, CYb);
	

	Vec3 aero_force(A * lbtoN, N * lbtoN, Y * lbtoN);
	Vec3 aero_force_pos(0, 0, 0);
	add_local_force(aero_force, aero_force_pos);

	L_roll			= rollmoment(Clb, Cldadot, Clda, Cldr, q, meanChord, wingArea, latStickInput, aileron_DEG, rudder_DEG, wingSpan, betaRad, rollRate_rps, rho, u, aoaRad, Clr, V_freeStream, pedalInput, yawRate_rps);
	M_pitch			= pitchmoment(Cmalpha, Cmde, Cmq, Cmadot, q, meanChord, wingArea, longStickInput, elevator_DEG, rudder_DEG, wingSpan, betaRad, pitchRate_rps, rho, u, aoaRad, V_freeStream, pedalInput, alphadot, trim_bias);
	N_yaw			= yawmoment(Cnb, Cnr, Cndr, Cnda, q, meanChord, wingArea, latStickInput, aileron_DEG, rudder_DEG, wingSpan, betaRad, yawRate_rps, rho, u, aoaRad, V_freeStream, pedalInput);

	Vec3 Moment_total(L_roll * ftlbtonm, N_yaw * ftlbtonm, M_pitch * ftlbtonm);
	add_local_moment(Moment_total);

	// Cockpit
	P0				   = calc_total_pressure(q_airspeedGauge, P);
	indicated_airspeed = calc_indicated_airspeed(V_indicated_airspeed, kgm3toslugft3, P0, P) * fpstoknts;
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
	aoaRad			= common_angle_of_attack;
	aoaRaw			= common_angle_of_attack * radiansToDegrees;
	betaRaw			= common_angle_of_slide * radiansToDegrees;
	betaRad		    = common_angle_of_slide;
	rollRate_rps	= omegax;
	pitchRate_rps	= omegaz;
	yawRate_rps		= -omegay;
	u				= vx * mpstofps;
}
/*
input handling
*/
void ed_fm_set_command (int command, float value)
{
	// Flight Control
	if (command == inputs::JoystickThrottle)//iCommandPlaneThrustCommon
	{
		throttle = 0.5 * (-value + 1.0);
	}

	if (command == inputs::JoystickPitch)//iCommandPlanePitch
	{
		longStickInput = limit(value, -1.0, 1.0);
	}

	if (command == inputs::JoystickRoll)//iCommandPlaneRoll
	{
		latStickInput = limit(value, -1.0, 1.0);
	}

	if (command == inputs::JoystickYaw)//iCommandPlaneYaw
	{
		pedalInput = limit(value, -1.0, 1.0);
	}

	if (command == inputs::TrimUp)//TrimUp
	{
		trim_bias = limit(trim_bias + 0.002, -0.3, 0.3);
	}

	if (command == inputs::TrimDown)//TrimDown
	{
		trim_bias = limit(trim_bias - 0.002, -0.3, 0.3);
	}

	if (command == inputs::TrimNeutral)//TrimNeutral
	{
		trim_bias = 0.0;
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
		delta_mass_pos_x = -1.0;
		delta_mass_pos_y =  1.0;
		delta_mass_pos_z =  0;

		delta_mass_moment_of_inertia_x	= 0;
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
	set internal fuel volume , init function, called on object creation and for refueling , 
	you should distribute it inside at different fuel tanks
*/
void   ed_fm_set_internal_fuel(double fuel)
{
	internal_fuel = fuel;
}
/*
	get internal fuel volume 
*/
double ed_fm_get_internal_fuel()
{
	return internal_fuel;
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

void ed_fm_set_draw_args (EdDrawArgument * drawargs,size_t size)
{
	drawargs[11].f   = (float)(-latStickInput * 3);
	drawargs[12].f	 = (float)(latStickInput * 3);

	drawargs[13].f	 = (float)(longStickInput * 3);

	drawargs[14].f	 = (float)(-pedalInput * 3);

	drawargs[15].f	 = (float)(trim_bias * 10);

	if (size > 616)
	{	
		drawargs[611].f = drawargs[0].f;
		drawargs[614].f = drawargs[3].f;
		drawargs[616].f = drawargs[5].f;
	}

}

void ed_fm_set_fc3_cockpit_draw_args(EdDrawArgument * drawargs, size_t size)
{
	drawargs[1].f	 = (float)(-longStickInput * 3);
	drawargs[2].f	 = (float)(latStickInput * 3);
	drawargs[3].f	 = (float)(throttle * 3);
	drawargs[4].f	 = (float)(pedalInput * 3);
	drawargs[5].f	 = (float)(indicated_airspeed/260);
	drawargs[10].f	 = (float)(-trim_bias * 2.0733);
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
		/*case ED_FM_ENGINE_0_RPM:			
		case ED_FM_ENGINE_0_RELATED_RPM:	
		case ED_FM_ENGINE_0_THRUST:			
		case ED_FM_ENGINE_0_RELATED_THRUST:	
			return 0; // APU
		case ED_FM_ENGINE_1_RPM:
			return throttle * 3000;
		case ED_FM_ENGINE_1_RELATED_RPM:
			return throttle;
		case ED_FM_ENGINE_1_THRUST:
			return throttle * 5000 * 9.81;
		case ED_FM_ENGINE_1_RELATED_THRUST:
			return throttle;*/

		case ED_FM_ENGINE_1_CORE_RPM:
			// I think this should be void
		case ED_FM_ENGINE_1_CORE_RELATED_RPM:
			// I think this should be void
		case ED_FM_ENGINE_1_CORE_THRUST:
			//return 10000;
		case ED_FM_ENGINE_1_CORE_RELATED_THRUST:
			//return .68 + (throttle * .32);

		case ED_FM_PROPELLER_1_RPM:					// propeller RPM , for helicopter it is main rotor RPM
			//return (.68 + (throttle * .32)) * 2750;
		case ED_FM_PROPELLER_1_PITCH:				// propeller blade pitch
		case ED_FM_PROPELLER_1_TILT:				// for helicopter
		case ED_FM_PROPELLER_1_INTEGRITY_FACTOR:   // for 0 to 1 , 0 is fully broken ,
			//return 1;
		case ED_FM_PISTON_ENGINE_1_MANIFOLD_PRESSURE:
			//return throttle;
		default:
			break;
		}
	}
	else if (index >= ED_FM_SUSPENSION_0_RELATIVE_BRAKE_MOMENT &&
			 index < ED_FM_OXYGEN_SUPPLY)
	{
		static const int block_size = ED_FM_SUSPENSION_1_RELATIVE_BRAKE_MOMENT - ED_FM_SUSPENSION_0_RELATIVE_BRAKE_MOMENT;
		switch (index)
		{
		case 0 * block_size + ED_FM_SUSPENSION_0_GEAR_POST_STATE:
		case 1 * block_size + ED_FM_SUSPENSION_0_GEAR_POST_STATE:
		case 2 * block_size + ED_FM_SUSPENSION_0_GEAR_POST_STATE:
			return test_gear_state;
		}
	}

	/*switch (index)
	{
	case STICK_PITCH:
		return longStickInput;
	}

	return 0;*/

}


void ed_fm_cold_start()
{

}

void ed_fm_hot_start()
{

}

void ed_fm_hot_start_in_air()
{

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


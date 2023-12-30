#include "stdafx.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include <stdlib.h> 

double BrakeLeft = 0.0;
double BrakeRight = 0.0;

double throttle = 0.0;

static double door_speed = 0.01;

double driver_door_pos = 0.0;
double driver_door_target = 0.0;

double passenger_door_pos = 0.0;
double passenger_door_target = 0.0;

double left_door_pos = 0.0;
double left_door_target = 0.0;

double right_door_pos = 0.0;
double right_door_target = 0.0;

double tail_gate_pos = 0.0;
double tail_gate_target = 0.0;

double window_FL_pos = 0.0;
double window_FR_pos = 0.0;
double window_RL_pos = 0.0;
double window_RR_pos = 0.0;
double window_canopy_pos = 0.0;

double shifter_pos = 0.0;
double key_pos = 0.0;

bool acc = false;
bool ignition = false;

double engine_power = 0.0;

double speedo = 0.0;

double hazzard_switch = 0.0;
double hazzard_time = 0.0;
double hazzard_light_state = 0.0;

double brakeLights = 0.0;

double headlight_animation = 0.0;
double headlight_switch = 0.0;

double lowBeams = 0.0;
double hiBeams = 0.0;

double highBeamToggle = 0.0;
double highBeamFlash = 0.0;
double blinker_switch = 0.0;
double blinker_beams_pos = 0.0;

double blinkerLeft = 0.0;
double blinkerRight = 0.0;

double light_timer = 0.0;
bool blinker_on = false;
bool hazzard_on = false;

double dimmer_pos = 1.0;
double dimmer_animation = 0.0;

double throttle_key = 0.0;

double steering_target = 0.0;
double steering_pos = 0.0;

double turnleft = 0.0;
double turnright = 0.0;

double pedalInputArg = 0.0;
double pedalInput = 0.0;

double internal_fuel = 0;
double fuel_target = 0.0;
double fuel_total = 0.0;
double fuel_pos = 0.0;
bool first_start = false;

double oil_pos = 0.0;
double oil_target = 0.0;

double battery_pos = 0.0;
double battery_target = 0.0;

double temp_pos = 0.0;
double temp_target = 0.0;

void updateOil()
{
	if (!acc)
	{
		oil_target = 0.0;
	}
	else oil_target = 180.0;

	if (oil_target < oil_pos)
	{
		oil_pos -= 0.95;
		oil_pos = limit(oil_pos, 0.0, 1000);
		std::cout << "- oil: " << oil_pos << std::endl;
	}
	else if (oil_target > oil_pos)
	{
		oil_pos += 1.95;
		oil_pos = limit(oil_pos, 0.0, 180.0);
		std::cout << "+ oil: " << oil_pos << std::endl;
	}

	if (!acc)
	{
		battery_target = 0.0;
	}
	else battery_target = 14.4;

	if (battery_target < battery_pos)
	{
		battery_pos -= 0.15;
		battery_pos = limit(battery_pos, 0.0, 1000);
		std::cout << "- bat: " << battery_pos << std::endl;
	}
	else if (battery_target > battery_pos)
	{
		battery_pos += 0.15;
		battery_pos = limit(battery_pos, 0.0, 180.0);
		std::cout << "+ bat: " << battery_pos << std::endl;
	}

	if (!acc)
	{
		temp_target = 0.0;
	}
	else temp_target = 60.4;

	if (temp_target < temp_pos)
	{
		temp_pos -= 0.75;
		temp_pos = limit(temp_pos, 0.0, 1000);
		std::cout << "- temp: " << temp_pos << std::endl;
	}
	else if (temp_target > temp_pos)
	{
		temp_pos += 0.75;
		temp_pos = limit(temp_pos, 0.0, 180.0);
		std::cout << "+ temp: " << temp_pos << std::endl;
	}
}

void updateFuel()
{
	if (!acc)
	{
		fuel_target = 0.0;
		first_start = true;
	}
	else fuel_target = internal_fuel;

	if(!ignition)
	{
		if (fuel_target < fuel_pos)
		{
			fuel_pos -= 0.95;
			fuel_pos = limit(fuel_pos, 0.0, internal_fuel);
			std::cout << "- fuel: " << fuel_pos << std::endl;
		}
		else if (fuel_target > fuel_pos)
		{
			fuel_pos += 0.95;
			fuel_pos = limit(fuel_pos, 0.0, internal_fuel);
			std::cout << "+ fuel: " << fuel_pos << std::endl;
		}
	}
	else fuel_pos = internal_fuel;

	//std::cout << "updateFuel: " << fuel_pos << std::endl;
}

void updateSteering()
{
	if (steering_target == -1.0 && steering_pos > -1.0)
	{
		steering_pos -= 0.002;
		steering_pos = limit(steering_pos, -1.0, 0.0);
		pedalInputArg = limit(steering_pos, -1.0, 1.0);
		pedalInput = limit(steering_pos, -1.0, 1.0);
		//std::cout << "steering_pos: " << steering_pos << std::endl;
	}
	else if (steering_target == 1.0 && steering_pos < 1.0)
	{
		steering_pos += 0.002;
		steering_pos = limit(steering_pos, 0.0, 1.0);
		pedalInputArg = limit(steering_pos, -1.0, 1.0);
		pedalInput = limit(steering_pos, -1.0, 1.0);
		//std::cout << "steering_pos: " << steering_pos << std::endl;
	}
	else if (steering_target == 0.0 && steering_pos < 0.0)
	{
		steering_pos += 0.002;
		steering_pos = limit(steering_pos, -1.0, 0.0);
		//std::cout << "plus: " <<  std::endl;
		pedalInputArg = limit(steering_pos, -1.0, 1.0);
		pedalInput = limit(steering_pos, -1.0, 1.0);
	}
	else if (steering_target == 0.0 && steering_pos > 0.0)
	{
		steering_pos -= 0.002;
		steering_pos = limit(steering_pos, 0.0, 1.0);
		//std::cout << "minus: " <<  std::endl;
		pedalInputArg = limit(steering_pos, -1.0, 1.0);
		pedalInput = limit(steering_pos, -1.0, 1.0);
	}
}

void updateDimmer()
{
	if ( acc && headlight_switch == 1.0)
	{
		dimmer_animation = dimmer_pos;
		//std::cout << "DIMMER: " << dimmer_pos << std::endl;
	}
	else dimmer_animation = 0.0;
}

void updateBeams()
{
	if (!acc)
	{
		lowBeams = 0.0;
		hiBeams = 0.0;
		return;
	}

	if (acc && headlight_switch == 0.0 && highBeamFlash == 1.0)
	{
		hiBeams = 1.0;
		headlight_animation = 1.0;
	}
	else if (acc && headlight_switch == 0.0 && highBeamFlash == 0.0)
	{
		hiBeams = 0.0;
		headlight_animation = 0.0;
	}

	if (acc && headlight_switch == 1.0)
	{
		if (highBeamFlash == 1.0)
		{
			hiBeams = 1.0;
			lowBeams = 0.0;
			//blinker_beams_pos = 1.0;
			highBeamToggle = 0.0;
		}
		else if (highBeamToggle == 1.0)
		{
			hiBeams = 1.0;
			lowBeams = 0.0;
			//blinker_beams_pos = -1.0;
		}
		else
		{
			hiBeams = 0.0;
			lowBeams = 1.0;
			//blinker_beams_pos = 0.0;
		}
	}
	else lowBeams = 0.0;
	//std::cout << "BEAMS" << std::endl;
}

void updateHeadlightState()
{
	if (acc)
	{
		if (headlight_switch == 1.0)
		{
			headlight_animation = 1.0;
		}
		else headlight_animation = 0.0;
	}
	else headlight_animation = 0.0;
}

void updateBrakeLights()
{
	if (acc && shifter_pos != 0.0)
	{
		if (BrakeLeft > 0.0 || BrakeRight > 0.0)
		{
			brakeLights = 1.0;
		}
		else brakeLights = 0.0;
	}
	else brakeLights = 0.0;
}

void updateHazzardLights()
{

	//else if (blinker_on && (hazzard_switch == 0.0 || blinker_switch == 0.0))
	//{
	//	light_timer = 0.0;
	//	hazzard_light_state = 0.0;
	//
	//std::cout << blinker_on << std::endl;

	if (hazzard_switch == 0.0 && blinker_switch == 0.0) light_timer = 0.0;

	//if (blinker_on && !acc)
	//{
	//	blinkerLeft = 0.0;
	//	blinkerRight = 0.0;
	//	blinker_on = false;
	//	light_timer = 0.0;
	//	std::cout << "OFF" << std::endl;
	//}

	//Reset timer
	if (light_timer >= 1.0 || (hazzard_switch == 0.0 && hazzard_on) || (blinker_switch == 0.0 && blinker_on))
	{
		hazzard_on = false;
		blinker_on = false;
		blinkerLeft = 0.0;
		blinkerRight = 0.0;
		light_timer = 0.0;
		//std::cout << "RESET TIMER" << std::endl;
	}

	//Timer
	if (hazzard_switch == 1.0 || (acc && blinker_switch != 0.0))
	{
		light_timer += 0.006;
		light_timer = limit(light_timer, 0, 1);
		//std::cout << "TIMER" << std::endl;
	}

	//Light
	if (light_timer > 0.5)
	{
		if (hazzard_switch == 1.0)
		{
			blinkerLeft = 1.0;
			blinkerRight = 1.0;
		}
		else if (hazzard_switch == 0.0 && blinker_switch == 1.0)
		{
			blinkerRight = 1.0;
		}
		else if (hazzard_switch == 0.0 && blinker_switch == -1.0)
		{
			blinkerLeft = 1.0;
		}
	}
	else
	{
		blinkerLeft = 0.0;
		blinkerRight = 0.0;
		//std::cout << "OFF" << std::endl;
	}
	//std::cout << light_timer << std::endl;
}

void updateIgnition()
{
	if (key_pos == -1)
	{
		acc = false;
		ignition = false;
		//std::cout << "OFF" << std::endl;
	}
	else if(key_pos == 0)
	{
		acc = true;
		ignition = false;
		//std::cout << "ACC" << std::endl;
	}
	else if (key_pos == 1)
	{
		acc = true;
		ignition = true;
		//std::cout << "IGN" << std::endl;
	}
}

void updateShifter()
{

	if (shifter_pos == 0)
	{
		BrakeLeft = 1.0;
		BrakeRight = 1.0;
		//std::cout << "PARK" << std::endl;
	}
	else if (shifter_pos >= 9.0 && shifter_pos <= 11.0)
	{
		if(ignition) engine_power = throttle * -3000.0;
		//std::cout << "REVERSE" << std::endl;
	}
	else if (shifter_pos >= 19.0 && shifter_pos <= 21.0)
	{
		engine_power = 0.0;
		//std::cout << "NEUTRAL" << std::endl;
	}
	else if (shifter_pos >= 0.29 && shifter_pos < 31.0)
	{
		if (ignition) engine_power = (throttle + throttle_key) * 3000.0;
		//std::cout << "DRIVE" << std::endl;
	}
	else if (shifter_pos >= 0.39 && shifter_pos < 42.0)
	{
		if (ignition) engine_power = (throttle + throttle_key) * 3000.0;
		//std::cout << "3" << std::endl;
	}
	else if (shifter_pos >= 0.49 && shifter_pos < 52.0)
	{
		if (ignition) engine_power = (throttle + throttle_key) * 2000.0;
		//std::cout << "2" << std::endl;
	}
	else if (shifter_pos >= 0.59 && shifter_pos < 62.0)
	{
		if (ignition) engine_power = (throttle + throttle_key) * 10000.0;
		//std::cout << "1" << std::endl;
	}
}

void updateCanopyPos()
{
	window_canopy_pos = (window_FL_pos + window_FR_pos + window_RL_pos + window_RR_pos) / 4;
	//cout << "window_canopy_pos: " << window_canopy_pos << endl;
}

void updateDoors()
{
	if (driver_door_target == 1.0 && driver_door_pos < 1.0)
	{
		driver_door_pos += door_speed;
		driver_door_pos = limit(driver_door_pos, 0, 1.0);
		//std::cout << "+ POSITION: " << driver_door_pos << std::endl;
	}

	if (driver_door_target == 0.0 && driver_door_pos > 0.0)
	{
		driver_door_pos -= door_speed;
		driver_door_pos = limit(driver_door_pos, 0, 1.0);
		//std::cout << "- POSITION: " << driver_door_pos << std::endl;
	}

	if (passenger_door_target == 1.0 && passenger_door_pos < 1.0)
	{
		passenger_door_pos += door_speed;
		passenger_door_pos = limit(passenger_door_pos, 0, 1.0);
		//std::cout << "+ POSITION: " << passenger_door_pos << std::endl;
	}

	if (passenger_door_target == 0.0 && passenger_door_pos > 0.0)
	{
		passenger_door_pos -= door_speed;
		passenger_door_pos = limit(passenger_door_pos, 0, 1.0);
		//std::cout << "- POSITION: " << passenger_door_pos << std::endl;
	}

	if (left_door_target == 1.0 && left_door_pos < 1.0)
	{
		left_door_pos += door_speed;
		left_door_pos = limit(left_door_pos, 0, 1.0);
		//std::cout << "+ POSITION: " << left_door_pos << std::endl;
	}

	if (left_door_target == 0.0 && left_door_pos > 0.0)
	{
		left_door_pos -= door_speed;
		left_door_pos = limit(left_door_pos, 0, 1.0);
		//std::cout << "- POSITION: " << left_door_pos << std::endl;
	}

	if (right_door_target == 1.0 && right_door_pos < 1.0)
	{
		right_door_pos += door_speed;
		right_door_pos = limit(right_door_pos, 0, 1.0);
		//std::cout << "+ POSITION: " << right_door_pos << std::endl;
	}

	if (right_door_target == 0.0 && right_door_pos > 0.0)
	{
		right_door_pos -= door_speed;
		right_door_pos = limit(right_door_pos, 0, 1.0);
		//std::cout << "- POSITION: " << right_door_pos << std::endl;
	}
}
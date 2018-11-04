float hdg_to_plane = 0.0, distance_to_plane = 100, delta_height = 0.0;
float curr_hdg_GS =0.0, curr_hdg_encoder_GS = 0.0, p_pan = 0.0;
float last_pan = 0.0;
float last_tilt = 0.0;
float subtrim_pan = 0;
float subtrim_tilt = 0;

//required tilt and pan angles (coordinates)
float req_tilt = 0.0;
float req_pan = 0.0;
float d_pan = 0.0; // delta pan

void calc_plane_tracking_coordinates()
{

#ifdef DO_TRACKING
	float curr_hdg_to_plane;
	/*
	Compute heading to airplane
	*/
	float lat1 = lat_GCS / 10000000.0;
	float lat2 = lat_airplane / 10000000.0;
	float long1 = lon_GCS / 10000000.0;
	float long2 = lon_airplane / 10000000.0;

	
	//point the tracker to the airplane, if the communication is Ok
	if (status_airplane_telemetry == AIRPLANE_TELE_OK)
	{
		hdg_to_plane = course_to(lat1, long1, lat2, long2);
		distance_to_plane = distance_between(lat1, long1, lat2, long2);
		delta_height = alt_airplane - alt_GCS;
	}

	if (hdg_to_plane <= 180)
		curr_hdg_to_plane = hdg_to_plane;
	else
		curr_hdg_to_plane = hdg_to_plane - 360;

	//update the required tilt angle
	req_tilt = atan2(delta_height, distance_to_plane) * (180 / PI);
	
	req_tilt = max(0, min(90, req_tilt));
	//update the required pan angle
	req_pan = curr_hdg_to_plane;

	real_control_timer.lastUpdate = micros();
	coordinatesUnpdated = false;
#endif // DO_TRACKING
}

void PID_control()
{
	
	//make sure that the current ground control station heading angle is betwee -180 to 180 degrees
	if (hdg_GS <= 180)
		curr_hdg_GS = hdg_GS;
	else
		curr_hdg_GS = hdg_GS - 360;

	if (encoder_heading <= 180)
		curr_hdg_encoder_GS = encoder_heading;
	else
		curr_hdg_encoder_GS = encoder_heading - 360;

	//proportional delta between current heading and required heading
	//p_pan = curr_hdg_GS - req_pan;
	p_pan = curr_hdg_encoder_GS - req_pan + subtrim_pan;

	//make sure that the delta pan angle is between -180 to 180 degrees
	if (p_pan < -180)
		p_pan += 360;
	if (p_pan > 180)
		p_pan -= 360;

	//make sure tilt is between -10 to +90 degrees
	req_tilt = max(-10, min(90, req_tilt + subtrim_tilt));

	// Compute the delta between the required heading and the current heading.
	//d_pan = (req_pan - last_pan) / (real_control_timer.deltat / 1000);

	if (DEBUG_CONTROL) {
		//port_Debug.print("["+ String(lat1,6) +"," + String(long1,6) +  "] --> [" + String(lat2,6) + "," + String(long2,6) + "] ");
		port_Debug.println("current heading: " + String(curr_hdg_GS, 2));
		port_Debug.println("require heading: " + String(req_pan, 2));
		port_Debug.println("require tilt: " + String(req_tilt, 2));
		//port_Debug.println("hdg_to_plane: " + String(req_pan, 2));
		//port_Debug.println("delta_pan: " + String(delta_pan, 2));

		//port_Debug.println("distance_to_plane: " + String(distance_to_plane, 2));
		//port_Debug.println("delta_height: " + String(delta_height, 2));
		//port_Debug.println("d_pan: " + String(d_pan, 2));
		port_Debug.println();
	}

	//updates for computing deriviates
	last_pan = req_pan;
	last_tilt = req_tilt;

#ifdef DO_CONTROL

#ifndef TESTING
	//if (status_airplane_gps != AIRPLANE_GPS_OK)
	//	return;
	//;
#endif // !TESTING

		updateSmartTimer(&real_control_timer);
		stat_GCS_PID_rate.now += 1;

		int sign_p_pan = 0;
		if (p_pan < 0)
			sign_p_pan = -1;
		else
			sign_p_pan = 1;

		//DEBUG
		//port_Debug.println("p_pan: " + String(p_pan, 2));
		//port_Debug.println("Servo: " + String(max(-1000, min(sign_p_pan * 10 * Kp_pan * pow(abs(p_pan), 0.5) + 1 * Kd_pan * d_pan, 1000)), 2));

		//control
		servo_pan.write(middle_position - max(-1000, min(sign_p_pan*10 * Kp_pan * pow(abs(p_pan),0.75) + 1 * Kd_pan * d_pan, 1000)));
		servo_tilt.write(max_position - req_tilt * milisecs_per_deg);
#endif
}

/*
Compute the heading of the airplane, relative to the Antenna Tracker
*/
float course_to(float lat1, float long1, float lat2, float long2)
{
	// returns course in degrees (North=0, West=270) from position 1 to position 2,
	// both specified as signed decimal-degrees latitude and longitude.
	// Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
	// Courtesy of Maarten Lamers
	float dlon = radians(long2 - long1);
	lat1 = radians(lat1);
	lat2 = radians(lat2);
	float a1 = sin(dlon) * cos(lat2);
	float a2 = sin(lat1) * cos(lat2) * cos(dlon);
	a2 = cos(lat1) * sin(lat2) - a2;
	a2 = atan2(a1, a2);
	if (a2 < 0.0)
	{
		a2 += TWO_PI;
	}
	return degrees(a2);
}

float distance_between(float lat1, float long1, float lat2, float long2)
{
	// returns distance in meters between two positions, both specified 
	// as signed decimal-degrees latitude and longitude. Uses great-circle 
	// distance computation for hypothetical sphere of radius 6372795 meters.
	// Because Earth is no exact sphere, rounding errors may be up to 0.5%.
	// Courtesy of Maarten Lamers
	float delta = radians(long1 - long2);
	float sdlong = sin(delta);
	float cdlong = cos(delta);
	lat1 = radians(lat1);
	lat2 = radians(lat2);
	float slat1 = sin(lat1);
	float clat1 = cos(lat1);
	float slat2 = sin(lat2);
	float clat2 = cos(lat2);
	delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
	delta = sq(delta);
	delta += sq(clat2 * sdlong);
	delta = sqrt(delta);
	float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
	delta = atan2(delta, denom);
	return delta * 6372795;
}

/* Compute battery voltage and percentage left */
void getBatt()
{
	int measurments = 10;
	float sensorValue = 0;

	for (int i = 0; i < measurments; i++)
		sensorValue += analogRead(battery_sensor_pin) / measurments;

	battery_val = full_buttery_val * (sensorValue / max_read);

	if (DEBUG_SYS) {
		port_Debug.println("sensorValue = " + String(sensorValue, 2));
		port_Debug.println("Voltage = " + String(battery_val, 2));
	}
}


/*
Pan the AT by x degrees
params:
degree: int - degrees 0-360
*/
void pan_tilt_related()
{
		
	int pan  = myParser.getInt();
	int tilt = myParser.getInt();

	if(true)
		port_Debug.println("inc panning by " + String(pan) + " degrees, inc tilting by : " + String(tilt));

	//todo: pan the AT by x degrees  
	subtrim_pan += pan;
	subtrim_tilt += tilt;
}

/*
Pan the AT by x degrees
params:
degree: int - degrees 0-360
*/
void moveTo()
{

	int pan = myParser.getInt();
	int tilt = myParser.getInt();

	if (true)
		port_Debug.println("panning by " + String(pan) + " degrees, tilting by : " + String(tilt));

	//todo: pan the AT by x degrees  
	req_pan = pan;
	req_tilt = tilt;
}
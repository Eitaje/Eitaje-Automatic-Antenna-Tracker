#define AIRPLANE_GPS		0
#define GCS_COMPASS			1
#define GSC_BATT			2
#define GCS_GPS				3

//leds[0] = CHSV((i + 0) * 50, 255, 255);
boolean bazz_state = LOW;

void led_buzz_starting_sequence()
{
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < NUM_LEDS; j++)
			leds[j] = CRGB::White;

		// Show the leds
		FastLED.show();

		if(Buzzer_Enabled)
			digitalWrite(buzzer_pin, LOW);
		delay(50);

		for (int j = 0; j < NUM_LEDS; j++)
			leds[j] = CRGB::Black;

		// Show the leds
		FastLED.show();

		digitalWrite(buzzer_pin, HIGH);
		delay(50);
	}

}

void updateSysStateLeds()
{
	if (calibration_mode)
	{
		leds[GCS_GPS] = CRGB::Yellow;
		leds[GCS_COMPASS] = CRGB::Yellow;
		leds[AIRPLANE_GPS] = CRGB::Yellow;
		leds[GSC_BATT] = CRGB::Yellow;
		FastLED.show();
		return;
	}

	// status GCS GPS -------------------------------
	if (status_GCS_GPS == GCS_GPS_NO_COMMUNICATION)
		leds[GCS_GPS] = CRGB::Red;
	
	if (status_GCS_GPS == GCS_GPS_NO_LOCK)
		leds[GCS_GPS] = CRGB::Orange;

	if (status_GCS_GPS == GCS_GPS_OK)
		leds[GCS_GPS] = CRGB::Green;
	// status GCS GPS -------------------------------

	// status GCS Compass -------------------------------
	if (status_GCS_COMPASS == GCS_COMPASS_TIMEOUT)
		leds[GCS_COMPASS] = CRGB::Red;

	if (status_GCS_COMPASS == GCS_COMPASS_UNKNOWN)
		leds[GCS_COMPASS] = CRGB::Blue;

	if (status_GCS_COMPASS == GCS_COMPASS_OK)
		leds[GCS_COMPASS] = CRGB::Green;
	// status GCS Compass -------------------------------

	// status Airplane GPS ------------------------
	if (status_airplane_gps == AIRPLANE_GPS_TIMEOUT)
		leds[AIRPLANE_GPS] = CRGB::Red;

	if (status_airplane_gps == AIRPLANE_GPS_OK)
		leds[AIRPLANE_GPS] = CRGB::Green;
	
	if (status_airplane_gps == AIRPLANE_GPS_NO_DATA_YET)
		leds[AIRPLANE_GPS] = CRGB::Blue;

	if (status_airplane_gps == AIRPLANE_GPS_NO_LOCK)
		leds[AIRPLANE_GPS] = CRGB::Purple;
	// status GCS Compass -------------------------------

	//update the batt level led
	if (battery_val >= voltage_Q4)
	{
		leds[GSC_BATT] = CRGB::Green;
		FastLED.show();

		shortBeep(1);
		return;
		//silence();
	}
	else if (battery_val >= voltage_Q3) {
		leds[GSC_BATT] = CRGB::Yellow;
		FastLED.show();

		shortBeep(2);
		return;
	}
	else if (battery_val >= voltage_Q2) {
		leds[GSC_BATT] = CRGB::Orange;
		FastLED.show();

		shortBeep(3);
		return;
	}
	else if (battery_val >= voltage_Q1) {
		leds[GSC_BATT] = CRGB::Red;
		FastLED.show();

		shortBeep(4);
		return;
	}
	else if (battery_val < voltage_Q1)
	{
		//Danger Zone - Low Batt

		if (bazz_state == LOW) {
			bazz_state = HIGH;
			
			if(Buzzer_Enabled)
				digitalWrite(buzzer_pin, bazz_state);
			leds[GSC_BATT] = CRGB::DarkOrange;
			FastLED.show();
			delay(200);
			return;
		}
		else
		{
			bazz_state = LOW;
			if (Buzzer_Enabled)
				digitalWrite(buzzer_pin, bazz_state);
			leds[GSC_BATT] = CRGB::Red;

			FastLED.show();
			delay(200);
		}
	}


}

void shortBeep(int numBeeps)
{
	int i = 0;
	boolean beeperState = LOW;
	while (i < numBeeps*2)
	{
		updateSmartTimer(&Beeper_timer);
		if (Beeper_timer.deltat > beeper_delay) {
			Beeper_timer.lastUpdate = micros();
			if (Buzzer_Enabled)
				digitalWrite(buzzer_pin, beeperState);
			beeperState = !beeperState;
			i++;
		}
	}

}


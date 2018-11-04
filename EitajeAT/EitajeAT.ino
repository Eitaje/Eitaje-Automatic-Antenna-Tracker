/*
Name:       Eitaje AT.ino
Created:  7/21/2018 1:34:39 PM
Author:     EITAN-PC\Eitan
*/
#include "mavlink.h"
#include <Servo.h>
#include "System.h"
#include <MsgParser.h>
#include "FastLED.h"
#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(encoder_pinA, encoder_pinB);

//LEDS
// How many leds in your strip?
#define NUM_LEDS 4
CRGB leds[NUM_LEDS];
int ledState = LOW;             // ledState used to set the LED
int no_plane_heartbeat = 0;		// how many times we checked and found out there was no heartbeat from the plane

unsigned long previousMillis = 0;        // will store last time LED was updated

										 // constants won't change :
const long interval = 1000;           // interval at which to blink (milliseconds)

Servo servo_pan;  // create servo object to control a servo
Servo servo_tilt;  // create servo object to control a servo


				// MAVLink config
				/* The default UART header for your MCU */
int sysid = 1;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
int compid = 158;                ///< The component sending the message
int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing
unsigned int num_hbs_pasados, num_hbs = 10;

// Define the system type, in this case an airplane -> on-board controller
uint8_t system_type = MAV_TYPE_GENERIC;
uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
unsigned long previousMillisMAVLink = 0, next_interval_MAVLink = millis();

//list of strings. You can have as many as you want.
//each string is saved in flash memory only and does not take up any ram.
const char s0[] PROGMEM = "echo";	const char help0[] PROGMEM = "[a number]. Prints back the number.";
const char s1[] PROGMEM = "rlt";	const char help1[] PROGMEM = "[x]. Wifi communication";
const char s2[] PROGMEM = "pan";	const char help2[] PROGMEM = "[x]. Wifi communication";
//const char s0[] PROGMEM = "+IPD,0,11:pan";	const char help0[] PROGMEM = "[x]. Wifi communication";


//This is our look up table. It says which function to call when a particular string is received
const FuncEntry_t functionTable[] PROGMEM = {
	//  String, help, Function
{ s0, help0,   echo },
{ s1, help1,   pan_tilt_related},
{ s2, help1,   moveTo}
};

//this is the compile time calculation of the length of our look up table.
int funcTableLength = (sizeof functionTable / sizeof functionTable[0]);     //number of elements in the function table
MsgParser myParser;     //this creates our parser


void setup() {

	LEDS.addLeds<WS2812B, indication_del_pin, GRB>(leds, NUM_LEDS);
	LEDS.setBrightness(60);
	LEDS.clearData();

	//Batt sample resolution
	//This will return values from analogRead() between 0 and 4095.
	analogReadResolution(adc_bits);

	// set the digital pin as output:
	pinMode(debug_led_pin, OUTPUT);
	port_Debug.begin(DEBUG_BAUDRATE);
	port_MAVLINK.begin(MAVLINK_BAUDRATE);
	port_Wifi.begin(WIFI_BAUDRATE);

	pinMode(SERVO_pin_pan, OUTPUT);
	pinMode(SERVO_pin_tilt, OUTPUT);
	pinMode(buzzer_pin, OUTPUT);
	pinMode(current_sensor_pin, INPUT);
	pinMode(battery_sensor_pin, INPUT);
	pinMode(mode_switch_pin, INPUT);
	digitalWrite(buzzer_pin, HIGH);
	//more?

	// attached servos
	servo_pan.attach(SERVO_pin_pan);
	servo_tilt.attach(SERVO_pin_tilt);

	//Safety precaution (nullify throttle value):
	servo_pan.write(middle_position);
	servo_tilt.write(middle_position);

	//DEBUG
	//float tilt = 0;
	//servo_tilt.write(max_position - tilt * milisecs_per_deg);

	led_buzz_starting_sequence();

	establish_comm();

	myParser.setTable(functionTable, funcTableLength);      //tell the parser to use our lookup table
	myParser.setHandlerForCmdNotFound(commandNotFound);     //Tell the parser which function to call when 
	myParser.useStartByteSet(true);//(byte)'$');
	myParser.setEndByte((byte)'*');

	//Request specific data streams from Mavlink
	Mav_Request_Data();
}

void loop() {


#ifdef COMM_STATISTICS
	//print statistics
	updateSmartTimer(&stat_print_timer);
	if (stat_print_timer.deltat > statistics_print_every)
	{
		stat_print_timer.lastUpdate = micros();
		printStatistics();
	}
#endif // COMM_STATISTICS

	//Do control procedure each [PID_control_procedure_every] time
	updateSmartTimer(&control_timer);
	if (control_timer.deltat > PID_control_procedure_every) {
		control_timer.lastUpdate = micros();

		if(coordinatesUnpdated)
			calc_plane_tracking_coordinates(); // re-compute where to point the AT to

		getEncoderPosition();
		PID_control(); // point the AT to the desired coordinates
	}
	

	
	updateSmartTimer(&sys_status_timer);

	//DEBUG
	//port_Debug.println("sys_status_timer.deltat: " + String(sys_status_timer.deltat));

	if (sys_status_timer.deltat > compute_sys_status_every) {
		sys_status_timer.lastUpdate = micros();


		if (DEBUG_STATUS)
			port_Debug.println("Updating AT status now");

		
		getBatt();
		updateSysStateLeds();
	}

	/*
	// switch debug led state:
	if (millis() - previousMillis >= interval) {
		// save the last time you blinked the LED
		previousMillis = millis();

		// if the LED is off turn it on and vice-versa:
		if (ledState == LOW) {
			ledState = HIGH;
		}
		else {
			ledState = LOW;
		}

		// set the LED with the ledState of the variable:
		digitalWrite(debug_led_pin, ledState);

		//Mavlink
		// Initialize the required buffers
		mavlink_message_t msg;
		uint8_t buf[MAVLINK_MAX_PACKET_LEN];

		// Pack the message
		mavlink_msg_heartbeat_pack(1, 0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);

		// Copy the message to the send buffer
		uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

		// Send the message with the standard UART send function
		// uart0_send might be named differently depending on
		// the individual microcontroller / library in use.
		unsigned long currentMillisMAVLink = millis();
		if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
			// Timing variables
			previousMillisMAVLink = currentMillisMAVLink;
			next_interval_MAVLink += 1000;

			//write heartbeat
			port_MAVLINK.write(buf, len);
		}
	}*/


	// Check reception buffer ============================================================
	//comm_receive_GCS();
	//comm_receive_Airplane();
	// ===================================================================================

	//check timeouts ---------------------------------------------------------------------
	////// GCS mavlink (for self GPS and compass information)
	updateSmartTimer(&mavlink_heartbeat_timer);
	if (mavlink_heartbeat_timer.deltat > mavlink_heartbeat_timeout)
	{
		status_GCS_mavlink = GCS_MAVLINK_TIMEOUT;
	}
	else
		status_GCS_mavlink = GCS_MAVLINK_OK;

	// GCS GPS timeout?
	updateSmartTimer(&gcs_gps_timer);
	if (gcs_gps_timer.deltat > GPS_timeout)
	{
		status_GCS_GPS = GCS_GPS_NO_COMMUNICATION;
	}
	else
	{
		if (CGS_gps_got_3d_fix)
			status_GCS_GPS = GCS_GPS_OK;
		else
			status_GCS_GPS = GCS_GPS_NO_LOCK;
	}

	// GCS compass timeout?
	updateSmartTimer(&gcs_compass_timeout_timer);
	if (gcs_compass_timeout_timer.deltat > compass_timeout)
	{
		status_GCS_COMPASS = GCS_COMPASS_TIMEOUT;
	}
	else
	{
		status_GCS_COMPASS = GCS_COMPASS_OK;
	}

	////// Airplane mavlink timeout? (for airplane GPS information)
	updateSmartTimer(&airplane_telemetry_timer);
	if (airplane_telemetry_timer.deltat > airplane_telemetry_timeout)
	{
		no_plane_heartbeat++;
		status_airplane_telemetry = AIRPLANE_TELE_TIMEOUT;
		if(no_plane_heartbeat >=10)
			status_airplane_gps = AIRPLANE_GPS_TIMEOUT;
	}
	else {
		no_plane_heartbeat = 0;
		status_airplane_telemetry = AIRPLANE_TELE_OK;
		
	}

	//check timeouts ---------------------------------------------------------------------

	updateSmartTimer(&status_debug_timer);
	if (status_debug_timer.deltat >= status_debug_every)
	{
		status_debug_timer.lastUpdate = micros();
		if (DEBUG_STATUS)
			print_status_to_debugPort();
	}

	//DEBUG
	if (DEBUG_LOOP)
	{
		loopFreq++;
		updateSmartTimer(&loop_frequency_meter_timer);
		if (loop_frequency_meter_timer.deltat > 1000)
		{
			loop_frequency_meter_timer.lastUpdate = micros();
			port_Debug.println("Loop per sec: " + String(loopFreq / 1000.0, 2) + "k");
			loop_frequency_meter_timer.lastUpdate = loop_frequency_meter_timer.now;
			loopFreq = 0;
		}
	}

	updateSmartTimer(&modeSwitchsState_timer);
	if (modeSwitchsState_timer.deltat > mode_switch_read_every)
	{
		modeSwitchsState_timer.lastUpdate = micros();
		calibration_mode = digitalRead(mode_switch_pin);

		//debug
		if(DEBUG_STATUS)
			if(calibration_mode)
				port_Debug.println("Callibration mode on");
			else
				port_Debug.println("Callibration mode off");
			
	}
}

void getEncoderPosition()
{
	long newPosition = -myEnc.read();
	if ((newPosition/ 2.13456789) != encoder_heading) {
		encoder_heading = newPosition/2.13456789;
		if(DEBUG_ENCODER)
			port_Debug.println("Encoder: " + String(encoder_heading,2));
	}
}
void Mav_Request_Data()
{
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	// STREAMS that can be requested
	/*
	Definitions are in common.h: enum MAV_DATA_STREAM

	MAV_DATA_STREAM_ALL=0, // Enable all data streams
	MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
	MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
	MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
	MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
	MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
	MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
	MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
	MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
	MAV_DATA_STREAM_ENUM_END=13,

	Data in PixHawk available in:
	- Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
	- Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
	*/

	// To be setup according to the needed information to be requested from the Pixhawk
	const int  maxStreams = 2;
	const uint8_t MAVStreams[maxStreams] = { MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_RAW_SENSORS };
	const uint16_t MAVRates[maxStreams] = { 0x04, 0x01 };

	for (int i = 0; i < maxStreams; i++) {
		/*
		mavlink_msg_request_data_stream_pack(system_id, component_id,
		&msg,
		target_system, target_component,
		MAV_DATA_STREAM_POSITION, 10000000, 1);

		mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id,
		mavlink_message_t* msg,
		uint8_t target_system, uint8_t target_component, uint8_t req_stream_id,
		uint16_t req_message_rate, uint8_t start_stop)

		*/
		mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
		uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
		//port_MAVLINK.write(buf, len);
		port_Wifi.write(buf, len);
	}
}

/*
  Receive GCS data (self GPS and compass data)
*/
void serialEvent1() { //comm_receive_GCS
	mavlink_message_t msg;
	mavlink_status_t status;

	while (port_MAVLINK.available() > 0) {
		uint8_t c = port_MAVLINK.read();

		//update mavlink heartbeat timer
		mavlink_heartbeat_timer.lastUpdate = micros();

		// Try to get a new message
		if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {



			// Handle message
			switch (msg.msgid) {
			case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
			{
				//DEBUG
				//port_Debug.println("MAVLINK_MSG_ID_HEARTBEAT");

				// E.g. read GCS heartbeat and go into
				// comm lost mode if timer times out
			}
			break;

			case MAVLINK_MSG_ID_GPS_RAW_INT:
			{
				//update rate counters
				stat_GCS_GPS_rate.now += 1;

				//mavlink_message_t* msg;
				mavlink_gps_raw_int_t gps_raw_int;
				mavlink_msg_gps_raw_int_decode(&msg, &gps_raw_int);

				//@param fix_type 0 - 1: no fix, 2 : 2D fix, 3 : 3D fix.Some applications will not use the value of this
				if (gps_raw_int.fix_type == 3)
					CGS_gps_got_3d_fix = true;
				else
					CGS_gps_got_3d_fix = false;

				//DEBUG
				if (DEBUG_GCS_GPS) { //DEBUG_GCS_GPS
					port_Debug.println("GCS Num Sattelites: " + String(gps_raw_int.satellites_visible) + ", GCS_gps_got_3d_fix: " + String(CGS_gps_got_3d_fix));
				}
			}
			break;
			case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
			{
				//update rate counters
				stat_GCS_GPS_rate.now += 1;

				/* Message decoding: PRIMITIVE
				mavlink_msg_global_position_int_decode(const mavlink_message_t* msg, mavlink_global_position_int_t* global_position_int)
				*/
				//mavlink_message_t* msg;
				mavlink_global_position_int_t global_position_int;
				mavlink_msg_global_position_int_decode(&msg, &global_position_int);

				lat_GCS = global_position_int.lat;
				lon_GCS = global_position_int.lon;
				alt_GCS = global_position_int.alt / 1000;
				//hdg_GS = global_position_int.hdg;


				//update gcs_GPS_timer 
				gcs_gps_timer.lastUpdate = micros();

				float distance_to_plane = distance_between(lat_GCS, lon_GCS, lat_airplane, lon_airplane);
				if (distance_to_plane > 100000)
					status_GCS_GPS = GCS_GPS_NO_LOCK;
				else if (lat_GCS != 0 && lon_GCS != 0)
					status_GCS_GPS = GCS_GPS_OK;

				//DEBUG
				if (DEBUG_GCS_GPS) {
					port_Debug.print("AntennaTracker_GPS: ");
					port_Debug.print(lat_GCS);
					port_Debug.print(", ");
					port_Debug.print(lon_GCS);
					port_Debug.print(", ");
					port_Debug.println(alt_GCS);
				}

			}
			break;

			case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
			{

				/* Message decoding: PRIMITIVE
				mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
				*/
				//mavlink_message_t* msg;
				mavlink_sys_status_t sys_status;
				mavlink_msg_sys_status_decode(&msg, &sys_status);
			}
			break;

			case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
			{
				//DEBUG
				port_Debug.println("MAVLINK_MSG_ID_PARAM_VALUE");

				/* Message decoding: PRIMITIVE
				mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
				*/
				//mavlink_message_t* msg;
				mavlink_param_value_t param_value;
				mavlink_msg_param_value_decode(&msg, &param_value);
			}
			break;

			case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
			{
				//DEBUG
				//port_Debug.println("MAVLINK_MSG_ID_RAW_IMU");

				/* Message decoding: PRIMITIVE
				static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
				*/
				//				mavlink_raw_imu_t raw_imu;
				//				mavlink_msg_raw_imu_decode(&msg, &raw_imu);
			}
			break;

			case MAVLINK_MSG_ID_ATTITUDE:  // #30
			{
				//DEBUG
				if (DEBUG_GS)
					port_Debug.println("MAVLINK_MSG_ID_ATTITUDE");

				//update rate counters
				stat_GCS_heading_rate.now += 1;

				/* Message decoding: PRIMITIVE
				mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
				*/
				mavlink_attitude_t attitude;
				mavlink_msg_attitude_decode(&msg, &attitude);

				hdg_GS = attitude.yaw * 180 / PI;
				hdg_GS += hdg_cal;

				if (hdg_GS < 0)
					hdg_GS += 360;
				if (hdg_GS > 360)
					hdg_GS -= 360;

				//update compass status
				gcs_compass_timeout_timer.lastUpdate = micros();
				status_GCS_COMPASS = GCS_COMPASS_OK;

				if (DEBUG_GCS_COMPASS) {
					port_Debug.println("Antenna tracker heading is: " + String(hdg_GS, 2));
				}
			}
			break;


			default:
				break;
			}
		}
	}
}


/*
Receive airplane data (mainly airplane GPS)
*/
void serialEvent2() {

#ifdef TESTING
	while (port_Wifi.available() > 0) {
		char c = port_Wifi.read();
		
		if(DEBUG_WIFI)
			port_Debug.print(c);
		
		myParser.processByte(c);

	}

#else	
	mavlink_message_t msg;
	mavlink_status_t status;

	while (port_Wifi.available() > 0) { //calibration mode
		uint8_t c = port_Wifi.read();

		if (calibration_mode) {
			//process Java command
			myParser.processByte(c);
		}
		else
		{ // mavlink mode
			//DEUBG
			//	port_Debug.print(c);

			// Try to get a new message
			if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

				// Handle message
				switch (msg.msgid) {
				case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
				{
					//update mavlink heartbeat timer
					airplane_telemetry_timer.lastUpdate = micros();

					//DEBUG
					if (DEBUG_AIRPLANE)
						port_Debug.println("AIRPLANE_MAVLINK_MSG_ID_HEARTBEAT");

					//flip Airplane Mavlink keep alive led state


					// E.g. read GCS heartbeat and go into
					// comm lost mode if timer times out
				}
				break;

				case MAVLINK_MSG_ID_GPS_RAW_INT:
				{
					//update rate counters
					stat_airplane_GPS_rate.now += 1;

					//mavlink_message_t* msg;
					mavlink_gps_raw_int_t gps_raw_int;
					mavlink_msg_gps_raw_int_decode(&msg, &gps_raw_int);

					satellites_visible = gps_raw_int.satellites_visible;
					//@param fix_type 0 - 1: no fix, 2 : 2D fix, 3 : 3D fix.Some applications will not use the value of this
					if (satellites_visible >= 4 || gps_raw_int.fix_type == 3) {
						airplane_gps_got_3d_fix = true;
						status_airplane_gps = AIRPLANE_GPS_OK;
					}
					else {
						airplane_gps_got_3d_fix = false;
						status_airplane_gps = AIRPLANE_GPS_NO_LOCK;
					}

					if (gps_raw_int.lat != lat_airplane) {
						lat_airplane = gps_raw_int.lat;
						coordinatesUnpdated = true;
					}
					if (gps_raw_int.lon != lon_airplane) {
						lon_airplane = gps_raw_int.lon;
						coordinatesUnpdated = true;
					}

					//DEBUG
					if (DEBUG_AIRPLANE) { //DEBUG_GCS_GPS
						port_Debug.println("Airplane Num Sattelites: " + String(satellites_visible) + ", airplane gps_got_3d_fix: " + String(CGS_gps_got_3d_fix));
					}
				}
				break;
				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{

					//update rate counters
					stat_airplane_GPS_rate.now += 1;


					/* Message decoding: PRIMITIVE
					mavlink_msg_global_position_int_decode(const mavlink_message_t* msg, mavlink_global_position_int_t* global_position_int)
					*/
					//mavlink_message_t* msg;
					mavlink_global_position_int_t global_position_int;
					mavlink_msg_global_position_int_decode(&msg, &global_position_int);
					if (global_position_int.lat != lat_airplane) {
						lat_airplane = global_position_int.lat;
						coordinatesUnpdated = true;
					}
					if (global_position_int.lon != lon_airplane) {
						lon_airplane = global_position_int.lon;
						coordinatesUnpdated = true;
					}

					alt_airplane = global_position_int.alt / 1000;
					hdg_airplane = global_position_int.hdg;

					// update status of airplane's gps 
					if (lat_airplane == 0 && lon_airplane == 0)
						status_airplane_gps = AIRPLANE_GPS_NO_LOCK;

					//DEBUG
					if (DEBUG_AIRPLANE) {
						port_Debug.print("AIRPLANE_GPS: ");
						port_Debug.print(lat_airplane);
						port_Debug.print(", ");
						port_Debug.print(lon_airplane);
						port_Debug.print(", ");
						port_Debug.print(alt_airplane);
						port_Debug.print(", ");
						port_Debug.println(hdg_airplane);
					}

				}
				case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
				{
					//DEBUG
					//port_Debug.println("MAVLINK_MSG_ID_SYS_STATUS");


					/* Message decoding: PRIMITIVE
					mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
					*/
					//mavlink_message_t* msg;
	//				mavlink_sys_status_t sys_status;
	//				mavlink_msg_sys_status_decode(&msg, &sys_status);
				}
				break;

				case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
				{
					//DEBUG
					//port_Debug.println("MAVLINK_MSG_ID_PARAM_VALUE");

					/* Message decoding: PRIMITIVE
					mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
					*/
					//mavlink_message_t* msg;
	//				mavlink_param_value_t param_value;
	//				mavlink_msg_param_value_decode(&msg, &param_value);
				}
				break;

				case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
				{
					//DEBUG
					//port_Debug.println("MAVLINK_MSG_ID_RAW_IMU");

					/* Message decoding: PRIMITIVE
					static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
					*/
					//				mavlink_raw_imu_t raw_imu;
					//				mavlink_msg_raw_imu_decode(&msg, &raw_imu);
				}
				break;

				case MAVLINK_MSG_ID_ATTITUDE:  // #30
				{
					/* Message decoding: PRIMITIVE
					mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
					*/
					//				mavlink_attitude_t attitude;
					//				mavlink_msg_attitude_decode(&msg, &attitude);

									//            if (attitude.roll > 1) leds_modo = 0;
									//            else if (attitude.roll < -1) leds_modo = 2;
									//            else leds_modo = 1;

									//            float yaw1 = mavlink_msg_attitude_get_yaw(&msg);
					//				port_Debug.print("Yaw: "); port_Debug.println(attitude.yaw * 180 / PI);
				}
				break;


				default:
					break;
				}
			}
		}// mavlink mode
	}
#endif // ! TESTING
}



/*
void serialEventRun(void) {
if (port_MAVLINK.available()) serialEvent();
}

void serialEventRun5(void) {
if (port_Wifi.available()) serialEvent5();
}
*/
/*
void serialEvent() {

	while (port_MAVLINK.available()) {
		port_Debug.print((char)Serial.read());
	}
}

void serialEvent1() {

	while (port_Wifi.available()) {
		port_Debug.print((char)Serial1.read());
	}
}
*/

void updateSmartTimer(SmartTimer * timer)
{
	timer->now = micros();

	// Set integration time by time elapsed since last filter update
	timer->deltat = ((timer->now - timer->lastUpdate) / 1000.0f);
}

void print_status_to_debugPort()
{
	port_Debug.print("status_GCS_mavlink: ");
	if (status_GCS_mavlink == GCS_MAVLINK_NO_COMMUNICATION) port_Debug.println("GCS_MAVLINK_NO_COMMUNICATION");
	if (status_GCS_mavlink == GCS_MAVLINK_OK) port_Debug.println("GCS_MAVLINK_OK");
	if (status_GCS_mavlink == GCS_MAVLINK_TIMEOUT) port_Debug.println("GCS_MAVLINK_TIMEOUT");

	port_Debug.print("GCS_COMPASS_health: ");
	if (status_GCS_COMPASS == GCS_COMPASS_OK) port_Debug.println("GCS_COMPASS_OK");
	if (status_GCS_COMPASS == GCS_COMPASS_UNKNOWN) port_Debug.println("GCS_COMPASS_UNKNOWN");
	if (status_GCS_COMPASS == GCS_COMPASS_TIMEOUT) port_Debug.println("GCS_COMPASS_TIMEOUT");

	port_Debug.print("GCS_GPS_health: ");
	if (status_GCS_GPS == GCS_GPS_NO_COMMUNICATION) port_Debug.println("GCS_GPS_NO_COMMUNICATION");
	if (status_GCS_GPS == GCS_GPS_NO_LOCK) port_Debug.println("GCS_GPS_NO_LOCK");
	if (status_GCS_GPS == GCS_GPS_OK) port_Debug.println("GCS_GPS_OK");

	port_Debug.print("airplane_GPS_health: ");
	if (status_airplane_telemetry == AIRPLANE_TELE_OK) port_Debug.println("AIRPLANE_TELE_OK");
	if (status_airplane_telemetry == AIRPLANE_TELE_NO_COMMUNICATION_YET) port_Debug.println("AIRPLANE_TELE_NO_COMMUNICATION_YET");
	if (status_airplane_telemetry == AIRPLANE_TELE_UNKNOWN) port_Debug.println("AIRPLANE_TELE_UNKNOWN");
	if (status_airplane_telemetry == AIRPLANE_TELE_TIMEOUT) port_Debug.println("AIRPLANE_TELE_TIMEOUT");

	port_Debug.print("airplane_GPS_health: ");
	if (status_airplane_gps == AIRPLANE_GPS_OK) port_Debug.println("AIRPLANE_GPS_OK");
	if (status_airplane_gps == AIRPLANE_GPS_NO_DATA_YET) port_Debug.println("AIRPLANE_GPS_NO_DATA_YET");
	if (status_airplane_gps == AIRPLANE_GPS_TIMEOUT) port_Debug.println("AIRPLANE_GPS_TIMEOUT");


	//line break
	port_Debug.println();
	/*
	enum GCS_COMPASS_health status_GCS_COMPASS = GCS_COMPASS_UNKNOWN;
	enum GCS_GPS_health status_GCS_GPS = GCS_GPS_NO_LOCK;
	enum airplane_telemetry_health status_airplane_telemetry = AIRPLANE_TELE_NO_COMMUNICATION_YET;
	enum airplane_GPS_health status_airplane_gps = AIRPLANE_GPS_NO_DATA_YET;
	*/
}

//This function is called when the msgParser gets a command that it didn't handle.
void commandNotFound(uint8_t* pCmd, uint16_t length)
{
	Serial.print("Command not found: ");
	Serial.write(pCmd, length); //print out what command was not found
	Serial.println();           //print out a new line
}

void echo()
{
	int aNumber;
	aNumber = myParser.getInt();

	Serial.print("You passed in the number ");
	Serial.println(aNumber);
}

/*
Print some statistics regarding the rate of incomming comm (airplane GPS, attitude etc.)
*/
void printStatistics()
{
	float AP_GPS = 0, GCS_GPS = 0, GCS_hdg = 0, GCD_PID = 0;

	AP_GPS = (1000.0f*(stat_airplane_GPS_rate.now - stat_airplane_GPS_rate.lastUpdate)) / stat_print_timer.deltat;
	GCS_GPS = (1000.0f*(stat_GCS_GPS_rate.now - stat_GCS_GPS_rate.lastUpdate)) / stat_print_timer.deltat;
	GCS_hdg = (1000.0f*(stat_GCS_heading_rate.now - stat_GCS_heading_rate.lastUpdate)) / stat_print_timer.deltat;
	GCD_PID = (1000.0f*(stat_GCS_PID_rate.now - stat_GCS_PID_rate.lastUpdate)) / stat_print_timer.deltat;

	//print
	port_Debug.println("\rstatistics\n--------------");
	port_Debug.println("AP GPS rate: " + String(AP_GPS, 2) + "hz");
	port_Debug.println("GCS GPS rate: " + String(GCS_GPS, 2) + "hz");
	port_Debug.println("GCS HDG rate: " + String(GCS_hdg, 2) + "hz");
	port_Debug.println("GCS PID rate: " + String(GCD_PID, 2) + "hz\n");

	//update counters
	stat_airplane_GPS_rate.lastUpdate = stat_airplane_GPS_rate.now;
	stat_GCS_GPS_rate.lastUpdate = stat_GCS_GPS_rate.now;
	stat_GCS_heading_rate.lastUpdate = stat_GCS_heading_rate.now;
	stat_GCS_PID_rate.lastUpdate = stat_GCS_PID_rate.now;
}
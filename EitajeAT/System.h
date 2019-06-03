#ifndef _System_h
#define _System_h

enum GCS_mavlink_health { GCS_MAVLINK_NO_COMMUNICATION, GCS_MAVLINK_OK, GCS_MAVLINK_TIMEOUT};
enum GCS_COMPASS_health { GCS_COMPASS_OK, GCS_COMPASS_UNKNOWN, GCS_COMPASS_TIMEOUT };
enum GCS_GPS_health { GCS_GPS_NO_COMMUNICATION, GCS_GPS_NO_LOCK, GCS_GPS_OK };
enum airplane_telemetry_health { AIRPLANE_TELE_OK, AIRPLANE_TELE_NO_COMMUNICATION_YET, AIRPLANE_TELE_UNKNOWN, AIRPLANE_TELE_TIMEOUT };
enum airplane_GPS_health { AIRPLANE_GPS_OK, AIRPLANE_GPS_NO_DATA_YET, AIRPLANE_GPS_NO_LOCK, AIRPLANE_GPS_TIMEOUT };
enum airplane_MODE { AIRPLANE_DISARMED, AIRPLANE_ARMED };

//#define TESTING

//Print debug strings?
#define DEBUG			    false
#define DO_CONTROL			// use servos to move Antenna tracking 
#define DO_TRACKING			// compute desired pan and tilt given airplane GPS coordinates
//#define COMM_STATISTICS     // print communication statistics

//#define DEBUG_AP

#define DEBUG_LOOP			false
#define DEBUG_SYS			true
#define DEBUG_STATUS		false
#define DEBUG_GS			false
#define DEBUG_GCS_GPS		false
#define DEBUG_GCS_COMPASS   false
#define DEBUG_AIRPLANE		false
#define DEBUG_CONTROL		false
#define DEBUG_ENCODER		false
#define DEBUG_WIFI			false
#define DEBUG_ORIENT_SETUP	false  // magnetic north orientation procedure debug
#define Buzzer_Enabled		false


// Serial Usage
#define port_Debug		SerialUSB
#define port_MAVLINK	Serial1
#define port_Wifi	    Serial2

//Servo PWM output (digital pins)
#define SERVO_pin_pan		8
#define SERVO_pin_tilt		9

//Pins usage
#define debug_led_pin					13 //   Debug led
#define indication_del_pin				36 //   status indication leds
#define buzzer_pin						37 //   status indication leds
#define current_sensor_pin				A1 //   analogue pin
#define battery_sensor_pin				A11 //   analogue pin
#define encoder_pinA					24 // encoder pin A
#define encoder_pinB					25 // encoder pin B
#define mode_switch_pin					45 // control whether the wifi input goes to calibration (via GCS) or to airplane Mavlink parsing
// Compass Calibration
float hdg_cal = 83.6;
//Battery
#define BATT_TYPE_3S_LIPO
// #define BATT_TYPE_4S_LIFE
// #define BATT_TYPE_8S_NIMH

#define adc_bits 12 //sample resolution

#ifdef BATT_TYPE_4S_LIFE
#define full_buttery_val 13.4 //13.67
#define max_read 4095
#define voltage_Q4 13.2 // 3.3v cell
#define voltage_Q3 13	// 3.25v 
#define voltage_Q2 12.8 // 3.2v 
#define voltage_Q1 12	// danger zone 3.0 volt
#endif //BATT_TYPE_4S_LIFE

#ifdef BATT_TYPE_3S_LIPO
#define full_buttery_val 12.6 //13.67
#define max_read 3880
#define voltage_Q4 12.0 // 4v cell
#define voltage_Q3 11.1	// 3.7v 
#define voltage_Q2 10.5 // 3.5v 
#define voltage_Q1 10.2	// danger zone 3.4 volt
#endif //#ifdef BATT_TYPE_3S_LIPO


#ifdef BATT_TYPE_8S_NIMH
#define full_buttery_val 11.2
#define max_read 3475
#define voltage_Q4 10.0 // 4v cell
#define voltage_Q3 9.6	// 3.7v 
#define voltage_Q2 9 // 3.5v 
#define voltage_Q1 8	// danger zone 3.4 volt
#endif /* BATT_TYPE_8S_NIMH */

float battery_val = 0.0;
float battery_left = 100.0;
float batt_level = 0.0;

//status flags
enum GCS_mavlink_health status_GCS_mavlink = GCS_MAVLINK_NO_COMMUNICATION;
enum GCS_COMPASS_health status_GCS_COMPASS = GCS_COMPASS_UNKNOWN;
enum GCS_GPS_health status_GCS_GPS = GCS_GPS_NO_LOCK;
enum airplane_telemetry_health status_airplane_telemetry = AIRPLANE_TELE_NO_COMMUNICATION_YET;
enum airplane_GPS_health status_airplane_gps = AIRPLANE_GPS_NO_DATA_YET;
enum airplane_MODE airplaneMode = AIRPLANE_DISARMED;

//Servo parameters
#define middle_position		  1475 //general middle
#define third_position        1333
#define two_thirds_position   1666
#define min_position		  1150
#define max_position		  2080
#define servo_range			  600  //PWM milliseconds
#define milisecs_per_deg	  10.333
//*/

/* Sensors Error Compensation */
#define ROLL_ERROR 0
#define PITCH_ERROR 0
#define COURSE_ERROR 0
#define RSSI_MIN_ACCEPTALE_VALUE 10

// %%%%%%%%%%%%%%%%%%%%% Tasks Scheduling %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Time between two repeated tasks (milli seconds)
////  Sensors
#define readSelfGPS_every				500 
#define readSelfCompass_every			50
#define status_debug_every				1000
#define mode_switch_read_every			500

#define PID_control_procedure_every		10
#define compute_sys_status_every		1000 // 333
#define mavlink_heartbeat_timeout		2500 // 333
#define airplane_telemetry_timeout		2500 // 333
#define GPS_timeout						2500 // 333
#define compass_timeout					2500 // 333
#define beeper_delay					50
#define statistics_print_every			2000
#define orientationProcess_every		100


//#define toggleDisplayEvery   500 // flip what's being displayed every x loops

//Control constants (PID coefficients, see: https://en.wikipedia.org/wiki/Control_theory)
const float Kp_pan = 1.0;		// Roll
const float Ki_pan = 0.002;	// 
const float Kd_pan = 0.25;		// 
const float Kp_tilt = 0.55;	// Pitch
const float Ki_tilt = 0.005;	// 
const float Kd_tilt = 0.05;	// 
const int   null_region = 0;	//units: degree

// COMMUNICATION BAUD RATE
#define MAVLINK_BAUDRATE	57600
#define WIFI_BAUDRATE		115200
#define DEBUG_BAUDRATE		115200
//#define OSD_BAUDRATE		115200
//#define TELEMETRY_BAUDRATE	57600

// holds the system state ------------------------------------------------------------------------------------
// General
unsigned int loopFreq; // how many loops per second
unsigned int loopCounter;
unsigned long first_loop_startTime;

//AIRPLANE's GPS Data
int32_t lat_airplane = 0; ///< Latitude, expressed as * 1E7
int32_t lon_airplane = 0; ///< Longitude, expressed as * 1E7
int32_t alt_airplane = 0; ///< Altitude in meters, expressed as * 1000 (millimeters), above MSL
uint16_t hdg_airplane = 0; ///< Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
boolean CGS_gps_got_3d_fix		= false; // true iff GCS gps's got a good sattelite lock
boolean airplane_gps_got_3d_fix = false; // true iff airplane gps's got a good sattelite lock
unsigned short int satellites_visible = 0;
boolean coordinatesUpdated		  = false; //true iff new coordinates arrived from airplane (via mavlink)
boolean remote_setup_mode		  = false; //true iff setup button is pressed
boolean isNorthCalibrationProcess = true;  // true when calibrating the tracker orientation 
boolean gcs_hase_home_loc		  = false; // true if GCS got a home coordinates

int32_t lat_GCS = 312521140; ///< Latitude, expressed as * 1E7
int32_t lon_GCS = 347247436; ///< Longitude, expressed as * 1E7
float  alt_GCS = 227.800; ///< Altitude in meters, expressed as * 1000 (millimeters), above MSL
float hdg_GS = 0; ///< Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
				 // holds the system state ------------------------------------------------------------------------------------
float encoder_heading = 0.0;

struct SmartTimer {
	unsigned long lastUpdate = 0, // used to calculate integration interval
		now = 0;
	float deltat = 0.0f;
};


//Timers for scheduling tasks
SmartTimer
status_debug_timer,
heartbeat_timer,
sys_status_timer,
control_timer,
real_control_timer,
loop_frequency_meter_timer,
Beeper_timer,
modeSwitchsState_timer,
mavlink_heartbeat_timer,
airplane_telemetry_timer,
gcs_compass_timeout_timer,
gcs_gps_timer,
stat_airplane_GPS_rate,
stat_GCS_heading_rate,
stat_GCS_GPS_rate,
stat_GCS_PID_rate,
stat_print_timer,
northCalibrationProcedureTimer
;
#endif
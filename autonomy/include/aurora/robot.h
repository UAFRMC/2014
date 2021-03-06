/**
 Aurora Robotics general robot code.
 
 This is shared between the Arduino and the PC.
 
  Orion Sky Lawlor, lawlor@alaska.edu, 2014-03-23 (Public Domain)
*/
#ifndef __AURORA_ROBOTICS__ROBOT_H
#define __AURORA_ROBOTICS__ROBOT_H
#include <stdint.h> /* for uint32_t */


/**
  These are the field dimensions, in centimeters.
  Y runs up the field, from collection bin to mining area.  It's always positive.
  X runs across the field, from left to right side.  It's signed.
*/
enum {
	field_y_size=738, // Y-length of field, in centimeters
//	field_y_size=500, // Y-length of field, in centimeters for the test arena
	field_y_start_zone=183, // y end of start area, in centimeters
	field_y_mine_zone=field_y_start_zone+294, // y start of mining area
//	field_y_mine_zone=field_y_start_zone+100, // y start of mining area for the test arena 
	
	field_x_size=378, // X-width of field, in centimeters
	field_x_hsize=field_x_size/2,
	field_x_GUI=field_x_hsize+10, // start X for GUI display
	field_x_bin=156, // X-width of collection bin, in centimeters
	field_x_hbin=field_x_bin/2
};

/** 
  This is a list of possible robot states.  
  It's mostly maintained on the backend, but
  can be commanded from the front end.
*/
typedef enum {
	state_STOP=0, ///< EMERGENCY STOP (no motion)
	state_drive, ///< normal manual driving

	state_autonomy, ///< full autonomy start state
	state_unfold, ///< deploy front wheels
	state_raise, ///< raise conveyor before driving
	state_find_camera, ///< turn until camera is visible
	state_align_turnout, ///< autonomous: pivot to face start position
	state_align_drive, ///< autonomous: initial drive to start position
	state_align_turnin, ///< autonomous: turn to face lunabin
	state_align_back, ///< autonomous: drive back to contact lunabin

	state_drive_to_mine, ///< autonomous: drive to mining area

	/* Semiauto mine mode entry point: */
	state_mine_lower, ///< mining mode: lowering head, driving forward
	state_mine_stall, ///< mining mode: raising head (after stall)
	state_mine, // actually mine
	state_mine_crouch, ///< exiting mining mode: lower front wheels (
	state_mine_raise, ///< existing mining mode: raise bucket

	state_drive_to_dump, ///< drive back to bin

	/* Semiauto dump mode entry point: */
	state_dump_contact, ///< final dock-and-dump mode: drive to contact bin
	state_dump_raise, ///< raising bucket
	state_dump_rattle, ///< rattle mode to empty bucket
	state_dump_lower, ///< lowering bucket (after dump)
	
	/* Semiauto dump mode entry point: */
	state_stow, // begin stowing: raise bucket
	state_stow_lower, // lower bucket
	state_stow_fold, // fold up front wheels
	state_stowed, // finished stowing (wait forever)

	state_last ///< end state (repeat from mine_drive)
} robot_state_t;
const char *state_to_string(robot_state_t state);


/// This bitfield convey's the robot's software status.
class robot_status_bits {
public:
	unsigned char stop:1; ///< EMERGENCY STOP engaged
	unsigned char arduino:1; ///< arduino is connected correctly
	unsigned char located:1; ///< robot thinks it knows where it is
	unsigned char autonomy:1; ///< full-autonomy mode is engaged
	unsigned char semiauto:1; ///< semiauto mode is engaged
};

/** This is the Arduino's AREF analog reference voltage.
  It's the scale factor that gives true voltage output,
  and should be measured from the AREF pin against Arduino ground. */
#define AD_AREF_voltage (4.90)

/** This scale factor converts an 
    Arduino Analog/Digital Data Number (0-1023) 
    to a real voltage, assuming direct feed-in. */
#define AD_DN2low_voltage (AD_AREF_voltage/(1023.0))

/** This scale factor converts an 
    Arduino Analog/Digital Data Number (0-1023) 
    to a real voltage, after the resistor divider scaling. 
   The constant is a weird fudge factor--the resistances alone don't seem to add up.
*/
#define AD_DN2high_voltage ((AD_AREF_voltage+0.9)*2000.0/(348.0*1023.0))


/** This class contains all the robot's sensors, on Arduino, backend, or front end.
Raw sensor values go as bitfields, because many of them are 10-bit quantities:
	- Arduino A/D values are 10 bits each
	- Arena positions in cm are 9-10 bits each (arena is 378x738cm)
	- Blinky angle reports are about 9 bits each (500 samples per rotation)
*/
class robot_sensors_arduino {
public:
	uint32_t battery:10; // raw A/D reading at top of battery stack (voltage = this*5*2000/384)
	uint32_t mining:10; // raw A/D value of mining head drive voltage (stalled when much less than battery)
	uint32_t bucket:10; // raw A/D value from dump bucket lift encoder
	
	uint32_t stalled:1; ///< mining head seems to be stalled out
	uint32_t stop:1; ///< EMERGENCY STOP button engaged
	
	uint32_t frontL:10; // raw A/D value from front-left wheel deploy encoder
	uint32_t frontR:10; // raw A/D value from front-right wheel deploy encoder
	uint32_t latency:10; // Arduino control loop latency
	
	uint32_t backL:1; ///< back left contact detected
	uint32_t backR:1; ///< back right contact detected

	uint32_t bucketFill:6; /// Bits for 6 infrared detectors
	uint32_t mineMoving:10; /// leaky-bucket, incremented on mine motion (200 is max motion; 0 is stopped)
	uint32_t avgStall:10; /// stall detection average
	uint32_t padding: 6;
};

/** This class contains "blinky" scanning infrared sensor reports. */
class robot_blinky_report {
public:
	uint32_t a0:10; // these should be an array, but you can't have a bitfield array
	uint32_t a1:10;
	uint32_t a2:10;
	uint32_t sync:2;
	 
	uint32_t a3:10;
	uint32_t a4:10;
	uint32_t a5:10;
	uint32_t timing:2; // low bits of millisecond counter
	
	enum {n_report=6};
	
	void write(int index,unsigned short value) {
		switch (index) {
		case 0: a0=value; break;
		case 1: a1=value; break;
		case 2: a2=value; break;
		case 3: a3=value; break;
		case 4: a4=value; break;
		case 5: a5=value; break;
		};
	}
	int read(int index) {
		switch (index) {
		case 0: return a0;
		case 1: return a1;
		case 2: return a2;
		case 3: return a3;
		case 4: return a4;
		case 5: return a5;
		};
		return -1000;
	}
};

/** This class contains robot localization information. */
class robot_localization {
public:
// Raw camera-derived robot location (cm)
	float x,y,z;

// Robot's orientation relative to lunabin (degrees from lunabin)
	float angle;

// Confidence in our position (1.0: recent detection; 0.0: no idea)
	float confidence;

// Blinky:
	uint32_t blinkL1:10; // angle of beacon 1 from left blinky detector
	uint32_t blinkL2:10; // angle of beacon 2 from left blinky detector
	uint32_t blinkR1:10; // angle of beacon 1 from right blinky detector
	uint32_t blinkR2:10; // angle of beacon 2 from right blinky detector
};


/**
 This class contains a power setting for each of the robot's actuators.
 
 The ":7" makes each a 7-bit field, with values:
 	1 (reverse) 
 	64 (stop) 
 	127 (forward)
*/
class robot_power {
public:
	enum { drive_stop=64 };

	unsigned char left:7; // left drive wheels
	unsigned char high:1; // High power mode
	
	unsigned char right:7; // right drive wheels
	unsigned char backMode:1; // Drive backwards (for final dump)
	
	unsigned char front:7; // deploy front drive wheels
	unsigned char frontUnused:1; // spare bit
	
	unsigned char mine:7; // mining head dig
	unsigned char mineMode:1; // if true, autonomously run mining head
	
	unsigned char dump:7; // storage bucket lift
	unsigned char dumpMode:1; // dock-and-dump mode 
	
	robot_power() { stop(); }
	void stop(void) {
		left=right=front=mine=dump=drive_stop; // all-stop
		high=dumpMode=mineMode=0;
	}
};

/**
 This class contains everything we currently know about the robot.
*/
class robot_current {
public:
	robot_state_t state; ///< Current control state
	robot_status_bits status; ///< Current software status bits
	robot_sensors_arduino sensor;  ///< Current hardware sensor values
	robot_localization loc; ///< Location
	robot_power power; // Current drive commands

	bool autonomous; 
};

#endif


/**
  Aurora Robotics Backend Code
  
  Orion Sky Lawlor, lawlor@alaska.edu, 2014-03-23 (Public Domain)
*/
#define AURORA_IS_BACKEND 1

#include <iostream>
#include <cmath>
#include "aurora/robot.h"
#include "aurora/robot.cpp"
#include "aurora/display.h"
#include "aurora/network.h"
#include "aurora/ui.h"
#include "aurora/robot_serial.h"

#include "SOIL/SOIL.h"

#include "ogl/event.cpp"
#include "osl/socket.cpp"

#include "osl/porthread.h" /* for threading */
#include "osl/porthread.cpp"

/** Video image analysis gets dumped here: */
#include "../aruco/viewer/location_binary.h"

#include "aurora/simulator.h"

bool simulate_only=false; // -sim flag
bool big_field=false; // -big flag

/**
 This class represents everything the back end knows about the robot.
*/
class robot_manager_t 
{
public:
	robot_current robot; // overall integrated current state
	
	robot_telemetry telemetry; // next-sent telemetry value
	robot_command command; // last-received command
	robot_comms comms; // network link to front end
	
	robot_serial arduino;

	robot_simulator sim;
	
	robot_manager_t() {
		// HACK: zero out main structures.
		//  Can't do this to objects with internal parts, like comms or sim.
		memset(&robot,0,sizeof(robot));
		memset(&telemetry,0,sizeof(telemetry));
		memset(&command,0,sizeof(command));
		
		// Start simulation in random real start location
		sim.loc.y=90.0;
		sim.loc.x=((rand()%2)-0.5)*2.0*84.5;
		sim.loc.angle=(rand()%6)*60;
		sim.front=0.4;
	}
	
	// Do robot work.
	void update(void);

private:
	// Autonomy support:
	double cur_time; // seconds since start of backend program
	double state_start_time; // cur_time when we entered the current state
	double mine_start_time; // cur_time when we last started mining
	double autonomy_start_time; // cur_time when we started full autonomy
	robot_state_t last_state;

	// Enter a new state (semi)autonomously
	void enter_state(robot_state_t new_state)
	{
		if (new_state==state_unfold) { autonomy_start_time=cur_time; }
		// if(!(robot.autonomous)) { new_state=state_drive; }
		
		// Log state timings to dedicate state timing file:
		static FILE *timelog=fopen("timing.log","w");
		fprintf(timelog,"%4d spent %6.3f seconds in %s\n",
			(int)(cur_time-autonomy_start_time),
			cur_time-state_start_time, state_to_string(robot.state));
		fflush(timelog);
		
		// Make state transition
		last_state=robot.state; // stash old state
		robot.state=new_state;
		robotPrintln("Entering new state %s",state_to_string(robot.state));
		state_start_time=cur_time;
	}

	// Advance autonomous state machine
	void autonomous_state(void);

	// Raw robot.power levels for various speeds
	enum {
		power_full_fw=127, // forward
		power_stop=64,
		power_full_bw=1 // backward
	};
	
	// Front wheel encoder target a/d values
	enum {
		front_mine=100, // mining crouches at most this much
		front_safe=90,  // can drive below this amount
		front_drive=41, // driving target value
		front_stow=700 // stowed value
	};
	
	// Dump bucket encoder target a/d values
	enum {
		bucket_mine_start=130, // start mining at this height
		bucket_drive=220, // normal driving height
		bucket_dump=780, // dumping height
		bucket_safe=300 // can safely drive below this height (without tipping over)
	};

	// Limit this value to lie in this +- range
	double limit(double v,double range) {
		if (v>range) return range;
		if (v<-range) return -range;
		else return v;
	}
	
	// Compare the front wheels to this value
	//  Returns true if our posture is OK (or failure)
	//    sign=1 (default):   front sensor < target
	//    sign=-1         :   front sensor > target
	bool front_posture(int front_dn_target,int sign=+1.0) {
		int diff=abs(robot.sensor.frontL-robot.sensor.frontR);
		if (diff>150) return true; // massive sensor mismatch--we're useless, so use open loop from here.
		if (robot.sensor.frontL*sign<front_dn_target*sign) return true;
		if (robot.sensor.frontR*sign<front_dn_target*sign) return true;
		return false;
	}
	
	/* Fail-false front wheel comparisons.  Idiomatic usage:
		if (front_above(front_...)) { ... }
	*/
	bool front_above(int dn) { return !front_posture(dn,+1.0); }
	bool front_below(int dn) { return !front_posture(dn,-1.0); }
	
	// Set the front wheels and bucket to natural driving posture
	//  Return true if we're safe to drive
	bool drive_posture() {
		robot.power.front=64+30; // weak front power to straighten wheels
		if (front_above(front_safe)) 
		{ // woa--we can't drive until front wheels deployed
			robot.power.front=power_full_fw;
			return false;
		}
		
		int tolerance=10; // dead zone (to prevent hunting)
		if (robot.sensor.bucket<bucket_drive-tolerance)
		{
			robot.power.dump=power_full_fw; // raise
			return false; // too low to drive yet
		}
		if (robot.sensor.bucket>bucket_drive+tolerance)
		{
			robot.power.dump=power_full_bw; // lower
		}
		return robot.sensor.bucket<bucket_safe;
	}
	
	// Autonomous driving rate:
	//  Returns 0-1.0 float power value.
	float drive_speed(float forward,float turn=0.0) {
		return 0.3; // confident but conservative
		// return 0.35; // problematic: spins too fast to see marker
	}

	// Autonomous feeler-based backing up: drive backward slowly until both switches engage.
	//  Return true when we're finally backed up properly.
	bool back_up()
	{
		// const int back_slow=64-(drive_speed(-1.0)*64);
		const int back_slow=64-(0.35*64); // faster back-up (hack!)
		if(!(drive_posture())) {return false;}
		else {
			if(robot.sensor.backL)
				{ robot.power.left=robot_power::drive_stop;}
			else 
				{ robot.power.left=back_slow;}
	
			if(robot.sensor.backR)
				{ robot.power.right=robot_power::drive_stop;}
			else
				{ robot.power.right=back_slow;}
		

			double time_in_state=fmod(cur_time-state_start_time,5.0);
			if (time_in_state>4.0 && time_in_state<4.5) { // stuck: drive forward
				robot.power.right=robot.power.left=64+(0.30*64);
			}
		
			return (robot.sensor.backL && robot.sensor.backR);
		}
	}
	
	// Autonomous driving: set powers to drive toward this field X,Y location 
	//  Returns true once we're basically at the target location.
	bool autonomous_drive(vec2 target) {
		if (!drive_posture()) return false; // don't drive yet
		
		double drive_power=drive_speed(+1.0);
		vec2 cur(sim.loc.x,sim.loc.y); // robot location
		double angle=sim.loc.angle; // degrees (!?)
		double arad=angle*M_PI/180.0; // radians
		vec2 orient(sin(arad),cos(arad)); // orientation vector (forward vector of robot)
		vec2 should=normalize(cur-target); // we should be facing this way
		
		double turn=orient.x*should.y-orient.y*should.x; // cross product (sin of angle)
		double drive=dot(orient,should); // dot product (like distance)
		
		double t=limit(turn*0.5,drive_power);
		double d=limit(drive*0.3,drive_power);
		double L=-d-t;
		double R=-d+t;
		robot.power.left=64+63*limit(L,0.5);
		robot.power.right=64+63*limit(R,0.5);
		
		return length(cur-target)<20.0; // we're basically there
	}
	
	// Force this angle (or angle difference) to be between -180 and +180, 
	//   by adding or subtracting 360 degrees.
	void reduce_angle(double &angle) {
		while (angle>=180) angle-=360; // reduce
		while (angle<-180) angle+=360; // reduce
	}
	
	// Autonomous turning: rotate robot so it's facing this direction.
	//  Returns true once we're basically at the target angle.
	bool autonomous_turn(double angle_target_deg=0.0,bool do_posture=true)
	{
		if (do_posture) { if (!drive_posture()) return false; } // don't drive yet
		double angle_err_deg=sim.loc.angle-angle_target_deg;
		reduce_angle(angle_err_deg);
		robotPrintln("Autonomous turn to %.0f from %.0f deg\n",
			angle_target_deg, sim.loc.angle);
		
		double turn=angle_err_deg*0.1; // proportional control
		double maxturn=drive_speed(0.0,1.0);
		turn=limit(turn,maxturn);
		robot.power.left=64-63*turn;
		robot.power.right=64+63*turn;
		return fabs(angle_err_deg)<5.0; // angle error tolerance
	}
	
	// Make sure we're still facing the collection bin.  If not, pivot to face it.
	bool check_angle() {
		if (robot.loc.confidence<0.2) return true; // we don't know where we are--just keep driving?
		double target=180.0/M_PI*atan2(robot.loc.x,robot.loc.y+200.0);
		double err=sim.loc.angle-target;
		robotPrintln("check_angle: cur %.1f deg, target %.1f deg",sim.loc.angle,target);
		reduce_angle(err);
		if (fabs(err)<10.0) return true; // keep driving--straight enough
		else return autonomous_turn(target,false); // turn to face target
	}
};

/** X,Y field location where we drive to before backing up */
vec2 dump_drive_loc(-40,50);

// Return true if the mining head is stalled (according to our sensors
bool is_stalled(const robot_current &robot) {
	if (robot.sensor.mineMoving<100) return true;
	// if (robot.sensor.mineMoving<100) return true;
	return false; // not stalled
}

// Return true if onboard storage is full of material
bool storage_bin_full(const robot_current &robot) {
	// if (robot.sensor.bucketFull&(1<<4)) return true; // up to level of sensor (FIXME: filter transients)
	return false; // not full
}

void robot_manager_t::autonomous_state()
{
	robot.power.stop(); // each state starts from scratch

// Drive constants
	float power_drive_float=drive_speed(1.0); // autonomous drive speed: 0.2 for concrete, 0.3 for dust (careful), 0.35 for fast autonomy
	int power_drive_fw=(int)(64+power_drive_float*63); // driving speed
	int power_stuck_fw=(int)(64+0.45*63); // un-stuck speed
	int power_drive_bw=(int)(64-power_drive_float*63);
	
	float robot_distance=sqrt(robot.loc.x*robot.loc.x+robot.loc.y*robot.loc.y);

	double time_in_state=cur_time-state_start_time;
	robotPrintln("In state %s for %.1f seconds...\n", state_to_string(robot.state), time_in_state);
	
	// full autonomy start
	if (robot.state==state_autonomy) {
		robot.autonomous=true;
		enter_state(state_unfold);
	}
	// unfold: deploy front wheels
	else if(robot.state==state_unfold)
	{
		if(time_in_state<15.0)  /// && front_above(front_drive))// seconds 
		{
			robot.power.front=power_full_fw;
		}
		else{ // done
			robot.power.front=power_stop;
			enter_state(state_raise);
		}
	}
	// raise: raise the mining head to clear ground for driving
	else if (robot.state==state_raise)
	{
		if(robot.sensor.bucket<bucket_drive && time_in_state<3.0)// raises until bucket_drive 
		{
			robot.power.front=power_full_fw; // keep deploying (in case it's not done)
			robot.power.dump=power_full_fw; // raise bin
		}
		else{ 
			robot.power.front=power_stop;
			robot.power.dump=power_stop;
			enter_state(state_find_camera);
		}
	}
	//state_find_camera: line up with centerline 
	else if (robot.state==state_find_camera)
	{
		if (!drive_posture()) { /* correct posture first */ }
		else if (robot.loc.confidence>0.5) { // we know where we are!
			sim.loc=robot.loc; // reset simulator to real detected values

			enter_state(state_align_turnout); 
		}
		else // don't know where we are yet--turn left
		{
			bool stop=false;
			if (time_in_state>20.0 && fmod(time_in_state,10.0)<1.5)
			{ // stop every 10 seconds (to avoid camera blur)
				stop=true;
			}
			
			if (!stop) {
				if (time_in_state<40.0) { // turn left
					robot.power.left=power_drive_bw;
					robot.power.right=power_drive_fw;
				}
				else if (time_in_state<80.0) { // stuck??? maybe turn right?
					robot.power.left=power_drive_fw;
					robot.power.right=power_drive_bw;
				}
				else { // HELP! 
					enter_state(state_drive);
				}
			}
		}
	}
	
	///< autonomous: rotate to face turning spot
	else if (robot.state==state_align_turnout) {
		double target_angle=-80.0; // degrees from Lunabin center
		if (sim.loc.x>0) target_angle=-target_angle; // right side? turn left.
		if (autonomous_turn(target_angle) || time_in_state>20.0) {
			enter_state(state_align_drive);
		}
	}
	
	///< autonomous: initial drive to line up with centerline
	else if (robot.state==state_align_drive) {
		bool x_aligned=fabs(sim.loc.x)<20.0; // on centerline
		if (!drive_posture()) { /* correct posture first */ }
		else if (x_aligned || time_in_state>20.0) { // done!
			enter_state(state_align_turnin);
		}
		else { // drive backward to turn area (stop if we hit wall)
			if (back_up()) enter_state(state_drive_to_mine); 
		}
	}
	
	///< autonomous: rotate to face directly toward bin center 
	else if (robot.state==state_align_turnin) {
		double target=0.0; // 180.0/M_PI*atan2(sim.loc.x,sim.loc.y);
		if (autonomous_turn(target) || time_in_state>20.0) {
			enter_state(state_align_back);
		}
	}
	
	///< autonomous: drive backwards to contact lunabin for initial orientation
	else if (robot.state==state_align_back) {
		if (back_up() || time_in_state>30.0)
		{ 
			enter_state(state_drive_to_mine);
		}
	}

	//state_drive_to_mine: Drive to mining area
	//TODO:Currently proportional drive. We can go high power to avoid obstacles
	else if (robot.state==state_drive_to_mine)
	{
		if (drive_posture()) {
			double target_Y=field_y_mine_zone+20; // mining area distance (plus buffer)
			double err_Y=target_Y-robot_distance;
			robot.power.left=power_drive_fw;
			robot.power.right=power_drive_fw;
			if (err_Y<0.0)  // we're there now
			{
				enter_state(state_mine_lower); // start mining!
			}
			check_angle();
			
			if (time_in_state>20.0) { // stuck?  high power mode!
				robot.power.left=robot.power.right=power_stuck_fw;
			}
		}
	}

	//Enter Semiauto mine modes

	//state_mine_lower: enter mining state
	else if (robot.state==state_mine_lower) {
		robot.power.dump=power_full_bw; // lower bucket
		if (robot.sensor.bucket<bucket_drive) { // start conveyor
			robot.power.mine=power_full_fw;
		}
		if (time_in_state<0.8) { // initial crouchmode 
			robot.power.front=power_full_bw; // down
		}
		else if(robot.sensor.bucket<bucket_mine_start || time_in_state>10.0) 
		{
			mine_start_time=cur_time; // update mine start time
			enter_state(state_mine);
		}
	}
	else if (robot.state==state_mine)
	{
		robot.power.mine=power_full_fw; // run mining head 
		
	//	if (front_above(front_mine)) { // lower down to mining state
	//		robot.power.front=power_full_fw; // lower mining head
	//	}
	
		double mine_time=cur_time-mine_start_time;
		if(	big_field || (
			robot_distance<field_y_size-50 && // field left to mine
			mine_time<35.0 &&  // time left to mine
			!storage_bin_full(robot) ))  // and there's room in the bin
		{ // keep mining
			check_angle();
			
		//	robot.power.mineMode=true; //Firmware takes care of mining and stall detection (BADLY!)

			if (robot.sensor.bucket<bucket_mine_start) {
			// ready to mine: slowly creep forward
				robot.power.left=64+6; // fix drift in software!
				robot.power.right=64+8;
			}
			
#if 0 // open-loop stall free version
			robot.power.dump=power_full_bw; // lower bucket hard
#else  // Software stall detection:
			robot.power.dump=64-34; // lower bucket (take deeper bite)
			
			if(is_stalled(robot)) { enter_state(state_mine_stall);}
#endif
		}
		else { enter_state(state_mine_crouch);} // done mining
	}

	// state_mine_stall: Detect mining head stall. Raise head until cleared
	else if (robot.state==state_mine_stall)
	{
		robot.power.mine=power_full_fw; // keep running mining head (so stall detection works)
		if(is_stalled(robot))
		{
			// if (robot.sensor.bucket<bucket_drive)  // fail!
			{
				robot.power.dump=power_full_fw; // lift head (only as high as driving zone)
			}
			robot.power.left=robot.power.right=64-3; // weakly drive backward (to roll out of hole)
		}
		else {enter_state(state_mine);} // not stalled? Then back to mining 
	}

	//state_mine_crouch: Raise front wheels at the end of mining area, to pivot off ground for lift
	else if (robot.state==state_mine_crouch)
	{
		robot.power.mineMode=false; //set mineMode bit to false
		// Raise front wheels.  FIXME FIXME!!! use front encoders here!
		if(time_in_state<1.5) //seconds
		{
			robot.power.front=power_full_bw;
		}
		else{ enter_state(state_mine_raise);}// raise mining conveyor
	}

	//state_mine_raise: Raise mining conveyor before starting to backup towards Lunarbin
	else if (robot.state==state_mine_raise)
	{
		if (robot.sensor.bucket>bucket_mine_start) {
			robot.power.front=power_full_fw;
		}
		if(robot.sensor.bucket<bucket_drive && time_in_state<10.0)
		{ 
			robot.power.dump=power_full_fw;
		}
		else
		{ 
			if (big_field) enter_state(state_drive);
			else enter_state(state_drive_to_dump);
		}
	}
		       
	// Drive back to bin 
	else if (robot.state==state_drive_to_dump)
	{
		if(time_in_state<5) //ReDeploy front wheels 
		{
			robot.power.front=power_full_fw;
		}
		if (time_in_state>0.5) // start driving backwards (overlaps with above)
		{
			autonomous_drive(dump_drive_loc); 
		}

		if (sim.loc.y<150) { enter_state(state_dump_contact);}
	}


	//Semiauto dump mode entry point: dock and dump mode
	else if (robot.state==state_dump_contact) // final backup to Lunarbin 
	{
		// robot.power.dump=power_full_fw; // lift head?  (fights drive posture)
		if (back_up() || time_in_state>30.0)
		{   
		  enter_state(state_dump_raise);
		}
	}

	// raise bucket to dump
	//FIXME: if linear actuators are not lifting then lower front wheels
	//FIXME: Halt for some time before switching states
	else if (robot.state==state_dump_raise)
	{
		if(robot.sensor.bucket<bucket_dump && time_in_state<45.0)
		{ robot.power.dump=power_full_fw;
		}
		else// bucket is high
		{
		  enter_state(state_dump_rattle); 
		}
	}
	// Give dust time to flow out (maybe gentle rattle?)
	else if (robot.state==state_dump_rattle)
	{
		robot.power.mine=power_full_fw; // empty out conveyor (and rattle)
		if(time_in_state>2.5) {
			enter_state(state_dump_lower);
		}
	}
	// lower bucket to safe driving height 
	else if (robot.state==state_dump_lower)
	{
		if(robot.sensor.bucket>bucket_safe  && time_in_state<20.0)
		{ robot.power.dump=power_full_bw;}
		else
		{ 
			if (big_field) enter_state(state_drive); // manual
			else enter_state(state_drive_to_mine);
		} // back to start again
	}
	else if (robot.state==state_stow)
	{
		robot.power.front=power_full_fw; // lower front wheels
		if (front_below(front_mine)) {
			robot.power.dump=power_full_fw; // raise bucket
		}
		if (robot.sensor.bucket>bucket_safe) {
			robot.power.mine=power_full_fw; // empty out conveyor (and rattle)
		}
		if (robot.sensor.bucket>bucket_dump || time_in_state>40.0)
		{
			enter_state(state_stow_lower);
		}
	}
	else if (robot.state==state_stow_lower)
	{
		if (robot.sensor.bucket>bucket_safe) {
			robot.power.mine=power_full_bw; // run conveyor backwards (clean-out)
		}
		
		robot.power.dump=power_full_bw; // lower
		if(robot.sensor.bucket<=bucket_mine_start  || time_in_state>20.0)
		{ enter_state(state_stow_fold);  }
	}
	else if (robot.state==state_stow_fold)
	{
		robot.power.dump=power_full_bw; // keep lowering
		robot.power.front=power_full_bw; // fold front wheels
		if(front_above(front_stow) || time_in_state>20.0)
		{ enter_state(state_stowed); }
	}
	else if (robot.state==state_stowed)
	{
		/* wait here forever */
	}
	else 
	{ // what?  unrecognized state?!  manual mode...
		robotPrintln("Autonomy: unrecognized state %s (%d)!?\n",state_to_string(robot.state), robot.state);
		enter_state(state_drive);
	}
}




robot_manager_t *robot_manager;

unsigned int video_texture_ID=0;

void robot_manager_t::update(void) {
	cur_time=0.001*glutGet(GLUT_ELAPSED_TIME);

// Check for an updated location from the "camera" vision application
	FILE *f=fopen("../aruco/viewer/marker.bin","rb");
	if (f) {
		static uint32_t last_count=-1;
		location_binary bin;
		if (1==fread(&bin,sizeof(bin),1,f) 
			&& bin.count!=last_count) 
		{ // file has been updated
			robotPrintln("Location updated to %.3f %.3f %.3f angle %.1f (%s)",
				bin.x,bin.y,bin.z,bin.angle,bin.valid?"valid":"invalid");
			
			// FIXME: add smoothing?
			if (bin.valid) {
				float marker_dx=0.0; // centimeters from bin center (X==0) to marker center (was -50 for marker to one side)
				float marker_dy=-60.0; // centimeters from bin lip (Y==0) to marker
				vec2 loc(-100.0*bin.x+marker_dx,100.0*bin.y+marker_dy); // center of camera
				
				float arad=bin.angle*M_PI/180.0; // radians
				vec2 right(cos(arad),-sin(arad)); // robot's right-hand axis
				float camera_right=+67.0; // centimeters from camera to robot center
				loc+=camera_right*right; // shift from camera to robot center
				
				robot.loc.x=loc.x;
				robot.loc.y=loc.y;
				robot.loc.z=100.0*bin.z;
				robot.loc.angle=bin.angle;
				robot.loc.confidence=std::max(robot.loc.confidence+0.2,0.0);
			}
			
			last_count=bin.count;
			
			static uint32_t last_vidcap=-1;
			if (bin.vidcap_count!=last_vidcap) {
				robotPrintln("Reading updated vidcap texture");
				video_texture_ID=SOIL_load_OGL_texture(
					"../aruco/viewer/vidcap.jpg",
					0,video_texture_ID,
					SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y);
				last_vidcap=bin.vidcap_count;
			}
		}
		fclose(f);
	}

	if (robotState_requested<state_last) {
		robot.state=robotState_requested;
		robotPrintln("Entering new state %s (%d) by backend UI request",
			state_to_string(robot.state),robot.state);
		robotState_requested=state_last; // clear UI request
	}
	
// Check for a command broadcast (briefly)
	int n;
	while (0!=(n=comms.available(10))) {
		if (n==sizeof(command)) {
			comms.receive(command);
			if (command.command==robot_command::command_STOP)
			{ // ESTOP command
				enter_state(state_STOP);
				robot.power.stop(); 
				robotPrintln("Incoming STOP command");
			}
			else if (command.command==robot_command::command_state)
			{
				if (command.state>=state_STOP && command.state<state_last)
				{
					robot.state=(robot_state_t)command.state;
					telemetry.ack_state=robot.state;
					robotPrintln("Entering new state %s (%d) by frontend request",
						state_to_string(robot.state),robot.state);
				} else {
					robotPrintln("ERROR!  IGNORING INVALID STATE %d!!\n",command.state);
				}
			}
			else if (command.command==robot_command::command_power)
			{ // manual driving power command
				robotPrintln("Incoming power command: %d bytes",n);
				if (robot.state==state_drive)
				{
					robot.autonomous=false;
					robot.power=command.power;
				}
				else 
				{
					robotPrintln("IGNORING POWER: not in drive state\n");
				}
			}
		} else {
			robotPrintln("ERROR: COMMAND VERSION MISMATCH!  Expected %d, got %d",
				sizeof(command),n);
		}
	}

// Perform action based on state recieved from FrontEnd
	//E-Stop command
	if(robot.state==state_STOP)
	{// All stop 
		robot.power.stop();
		state_start_time=cur_time;
	}
	else if (robot.state==state_drive)
	{ // do nothing-- already got power command
		state_start_time=cur_time;
	}
	else { // autonomous mode!
		autonomous_state();
	}

// Send commands to Arduino
	if (simulate_only) { // build fake arduino data
		robot.status.arduino=1; // pretend it's connected
		robot.sensor.backL=(sim.corner(false,false).y<0);
		robot.sensor.backR=(sim.corner(true,false).y<0);
		robot.sensor.bucket=sim.dump*(816-104)+104;
		robot.sensor.frontL=sim.front*900+31;
		robot.sensor.frontR=sim.front*700+19;
		robot.sensor.mineMoving=200;
	} else { // real arduino
		arduino.update(robot);
	}

// Send out telemetry
	static double last_send=0.0;
	if (cur_time>last_send+0.050)
	{
		last_send=cur_time;
		robotPrintln("Sending telemetry, waiting for command");
		telemetry.count++;
		telemetry.state=robot.state; // copy current values out for send
		telemetry.status=robot.status;
		telemetry.sensor=robot.sensor;
		telemetry.power=robot.power;
		telemetry.loc=robot.loc; robot.loc.confidence*=0.8;
		
		comms.broadcast(telemetry);
	}

// Show real and simulated robots
	robot_display(robot.loc);
	
	static double last_time=0.0;
	double dt=cur_time-last_time;
	if (dt>0.1) dt=0.1;
	last_time=cur_time;
	
	if (robot.loc.confidence>0.5)  // make sim track reality
		blend(sim.loc,robot.loc,robot.loc.confidence*dt);
	if (simulate_only) // make reality track sim
	{
		blend(robot.loc,sim.loc,0.1);
		if (fabs(sim.loc.angle)<40.0) // camera in view
			robot.loc.confidence+=0.1;
		else // camera not in view
			robot.loc.confidence*=0.9;
		robot.loc.confidence*=0.9;
		
		if (robot.loc.y>100 && robot.loc.y<500) { // simulate obstacles
		if ((rand()%1000)==0) { // crater!
			sim.loc.angle+=30.0;
		}
		if ((rand()%1000)==0) { // crater!
			sim.loc.angle-=30.0;
		}
		}
	}
	sim.simulate(robot.power,dt);
	
/*
	if (big_field) { // overwrite position with 0,0,0 (for semi-auto operation)
		robot.loc.x=0.0;
		robot.loc.y=0.6; // upfield
		robot.loc.z=0.0;
		robot.loc.angle=0;
		robot.loc.confidence=1.0;
		sim.loc=robot.loc;
	}
*/
	
	robot_display(sim.loc,0.5);
}


void display(void) {
	robot_display_setup(robot_manager->robot);
	
	robot_manager->update();
	
	glTranslatef(field_x_GUI+400.0,100.0,0.0);
	glScalef(300.0,200.0,1.0);
	glBindTexture(GL_TEXTURE_2D,video_texture_ID);
	glEnable(GL_TEXTURE_2D);
	glBegin(GL_QUAD_STRIP);
	glTexCoord2f(0.0,0.0); glVertex2f(0.0,0.0); 
	glTexCoord2f(1.0,0.0); glVertex2f(+1.0,0.0); 
	glTexCoord2f(0.0,1.0); glVertex2f(0.0,+1.0); 
	glTexCoord2f(1.0,1.0); glVertex2f(+1.0,+1.0); 
	glEnd();
	glBindTexture(GL_TEXTURE_2D,0);
	
	glutSwapBuffers();
	glutPostRedisplay();
}

int main(int argc,char *argv[]) 
{
	setenv("DISPLAY", ":0",1);
	glutInit(&argc,argv);
	
	// Set screen size
	int w=1280, h=720;
	for (int argi=1;argi<argc;argi++) {
		if (0==strcmp(argv[argi],"-sim")) { 
			simulate_only=true;
			if (argi+1<argc) srand(atoi(argv[++argi])); // optional seed argument 
			else srand(1);
		}
		else if (0==strcmp(argv[argi],"-big")) { 
			big_field=true;
		}
		else if (2==sscanf(argv[argi],"%dx%d",&w,&h)) {}
		else printf("Unrecognized argument '%s'!\n",argv[argi]);
	}
	glutInitDisplayMode(GLUT_RGBA + GLUT_DOUBLE);
	glutInitWindowSize(w,h);
	glutCreateWindow("Robot Backend");
	
	robot_manager=new robot_manager_t;
	robotMainSetup();
	
	glutDisplayFunc(display);
	glutMainLoop();
	return 0;
}


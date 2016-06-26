/**
  Aurora Robotics keyboard user interface code.
  
  Orion Sky Lawlor, lawlor@alaska.edu, 2014-03-23 (Public Domain)
*/
#ifndef __AURORA_ROBOTICS__UI_H
#define __AURORA_ROBOTICS__UI_H

#include "ogl/event.h" /* for joystick */

/**
 Keyboard-based user interface for robot.
*/
class robot_ui {
public:
	robot_power power; // Last output power commands
	
	// Current floating-point power values:
	float left, right, front, mine, dump;
	
	bool demo; // Used for demos, if true then only drive wheels work
	bool manualMine;	
	// Human-readable description of current state
	std::string description;
	
	void stop(void) {
		left=right=front=mine=dump=0.0;
		power.stop();
		description="Sending STOP";
	}
	
	// Respond to these keystrokes.
	//  The "keys" array is indexed by the character, 0 for up, 1 for down.
	void update(int keys[],const robot_current &robot);
	
	robot_ui() {
		stop();
		demo=false;
		manualMine=false;
		description="Starting up";
	}
	
	// Clamp this float within this maximum range
	float limit(float v,float maxPower) const {
		if (v>maxPower) v=maxPower;
		if (v<-maxPower) v=-maxPower;
		return v;
	}
	
	// Convert a raw float to a motor command, with this maximum range
	byte toMotor(float v,float maxPower) const {
		v=limit(v,maxPower);
		int iv=(int)(v*63+64);
		if (iv<1) iv=1;
		if (iv>127) iv=127;
		return iv;
	}
};

void robot_ui::update(int keys[],const robot_current &robot) {
	static int keys_last[256]; 
	int keys_once[256]; // key down only *one* time per press
	for (int i=0;i<256;i++) { 
		keys_once[i]= (keys[i] && !keys_last[i]);
		keys_last[i]=keys[i];
	}

// Power limits:
	float driveLimit=0.5;
	float frontLimit=1.0;
	float mineLimit=1.0;
	float dumpLimit=1.0;
	
// Prepare a command:
	if (keys[' ']) { // spacebar--full stop
		stop();
		robotState_requested=state_STOP;
		return; // don't do anything else.  Just stop.
	} 
	if(power.high)
	{
           description+=" ULTRA POWER";
           driveLimit=0.8;
	}
	// else spacebar not down--check other keys for manual control
	if(keys_once['D']) { demo=!demo;power.high=false;} // toggle restricted Demo mode
	description="Commands: ";
	if(demo){description+=" DEMO";}
/* scale back, so things die away when key is released.
   It'd be better to detect key-up here, and stop, but that's ncurses. */
        left*=0.80;
        right*=0.80;
        front*=0.5;
	mine*=0.5;
        dump*=0.5;
	
	
      
	static bool joyDrive=false;
	bool joyDone=false; // subtle: 
	

/*  Fix: Uses only the left analog stick for driving the robot */
	/* Uses the left analog stick*/
/* TODO: Map dumpMode to joystick*/
	float forward=-oglAxis(2,"Go Forward or Reverse"); // left Y
	float turn=0.9*oglAxis(1,"Turn Left or Right"); //left X, scaled for gentle turns
	
	/* Oerate the mining head*/
	 float dumpJoy=-oglAxis(3,"Operate Dump Buckets");
	 float mineJoy=oglAxis(4,"Run Mine Head");
	
	if(forward!=0.0 || turn!=0.0 || dumpJoy!=0 || mineJoy!=0) 
	{
		joyDrive=true; // using joystick
	}
	else {joyDone=true;}
	if(joyDrive)
	{
		left=driveLimit*(forward+turn);
		right=driveLimit*(forward-turn);
		/* Dont run mining head and dump while in demo mode*/
		if(!demo)
		{
		  dump=dumpLimit*dumpJoy;
		  mine+=mineJoy;
		}
		description += " joystick ";
	}
	joyDrive=!joyDone; 
	if (oglButton(3,"Stop")) 
	{ 
		stop(); 
		robotState_requested=state_STOP;
	}
	if(oglButton(4,"Drive"))
	{
		robotState_requested=state_drive;
	}
	if(!demo)
	{
	  if ((oglButtonOnce(1,"Break"))) { stop(); } // stop without changing state
	
	if (oglButton(5,"Deploy Front")) { front=1; description  +="frontJ+ "; }
	if(oglButton(6,"Raise Front")){ front-=1; description+="FrontJ-";}
	}
	
	
// Drive keys:
	float acceleration=.2;
        if(keys['w'])
        {
            left+=acceleration;
            right+=acceleration;
        }
        if(keys['s'])
        {
            left-=acceleration;
            right-=acceleration;
        }
        if(keys['a'])
        {
            left-=acceleration;
            right+=acceleration;
        }
        if(keys['d'])
        {
            left+=acceleration;
            right-=acceleration;
        }
        
	if (keys['A']) { /* crude key-based autonomy (state based is better) */
		if (robot.loc.confidence<0.5) description+=" (ROBOT LOST NO AUTONOMY!)";
		else {
			description+=" AUTONOMY";
			
			if (fabs(robot.loc.angle)>10.0) { // turn first
				description+="_TURN";
				double turnspeed=0.1;
				double dir=(robot.loc.angle>0.0)?+1.0:-1.0;
				left-=turnspeed*dir;
				right+=turnspeed*dir;
			}
			else { // lined up--drive!
				description+="_DRIVE";
				double target_Y=200.0; // cm distance
				double err_Y=target_Y-robot.loc.y;
				double dir=(err_Y>0.0)?+1.0:-1.0;
				if (fabs(err_Y)<15.0) dir=0.0; // we're there!
				double drivespeed=0.2;
				left+=drivespeed*dir;
				right+=drivespeed*dir;
			}
		}
	}
        

// Special features
if(!demo) { // only works if demo is false	
        if(keys['f'])
        {
            front=1;
            description+="front+ ";
        }
        if(keys['v'])
        {
            front=-1;
            description+="front- ";
        }
	
	
	/*if(keys_once['b'])
        {
	   power.backMode=!power.backMode;
        }*/
	if(keys_once['m'])
        {
	   power.mineMode=!power.mineMode;
	   mine=+1;
        }
	if(keys_once['n'])// mine continously without setting the miningMode bit
	{
	  manualMine=!manualMine;
	}
	if(keys['j'])
        {	
	    power.mineMode=false;
	    manualMine=false; 
            mine=-1;
            description+="mine- ";
        }
	if(keys_once['p'])
	{
	    power.high=!power.high;
	}	

        if(keys[oglSpecialDown])
        {
            dump=-1;
            description+="dump- ";
        }
        if(keys[oglSpecialUp])
        {
            dump=+1;
            description+="dump+ ";
        }
	/*if(keys_once[oglSpecialLeft])
	{
	    power.dumpMode=!power.dumpMode;
	}
	*/
	
	if(power.mineMode)
	{
	   description+=" MINEMODE+ ";
	   mine=+1.0;
	}
	/*if(power.backMode)
	{
	   description+=" BACKUPMODE+ ";
	   // SMELLS LIKE A HACK: what about autonomous mode?  What if frontend disconnects?
	   if (robot.sensor.backL && robot.sensor.backR) {
	   	power.backMode=false; // we're there!
	   	power.dumpMode=true; // keep dumping
	   }
	}*/
	
	if(power.dumpMode) { 
	   description+=" DUMPMODE";
	   dump+=1.0;
	}
	if(manualMine) { mine+=1;}
	
}	
	left=limit(left,driveLimit);
	right=limit(right,driveLimit);


	power.left=toMotor(left,driveLimit);
	power.right=toMotor(right,driveLimit);
	power.front=toMotor(front,frontLimit);
	power.mine=toMotor(mine,mineLimit);
	power.dump=toMotor(dump,dumpLimit);
}
	


#endif


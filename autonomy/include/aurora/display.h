/**
 Aurora Robotics OpenGL display code.
 Shared between front end and back end.
*/
#ifndef __AURORA_ROBOTICS__DISPLAY_H
#define __AURORA_ROBOTICS__DISPLAY_H

#include <GL/glut.h> /* OpenGL Utilities Toolkit, for GUI tools */

/* robotPrintln support code: */
#include <stdio.h>
#include <stdarg.h>  /* for varargs stuff below */
#include "osl/vec2.h"

robot_state_t robotState_requested=state_last;
vec2 robotMouse_pixel; // pixel position of robot mouse
vec2 robotMouse_cm; // field-coordinates position of mouse
bool robotMouse_down=false;

double robotPrintf_x=field_x_GUI, robotPrintf_y=0.0, robotPrintf_line=-25.0;

/* Return the current time, in seconds */
double robotTime(void) {
	return 0.001*glutGet(GLUT_ELAPSED_TIME);
}


bool robotPrintf_enable=true;
/* Render this string at this X,Y location */
void robotPrint(float x,float y,const char *str)
{
	if (robotPrintf_enable) {
        // Dump everything to the console, and log it too
	fprintf(stdout,"%.3f %s\n",robotTime(),str);
        fflush(stdout);
        
        static FILE *flog=fopen("log.txt","w");
	fprintf(flog,"%.3f %s\n",robotTime(),str);
        fflush(flog);
        }
        
        // Draw it onscreen
        void *font=GLUT_BITMAP_HELVETICA_12;
        glRasterPos2f(x,y);
        while (*str!=0) {
        	glutBitmapCharacter(font,*str++);
        	if (robotPrintf_enable && (*str=='\n' || *str==0)) { 
        		robotPrintf_x=field_x_GUI; 
        		robotPrintf_y+=robotPrintf_line; 
        	}
        }
        glPopAttrib();
        
}

/** Render this string onscreen, followed by a newline. */
void robotPrintln(const char *fmt,...) {
        va_list p; va_start(p,fmt);
        char dest[1000];
        vsnprintf(dest,sizeof(dest),fmt,p);
        robotPrint(robotPrintf_x,robotPrintf_y,dest);
        va_end(p);
}

/******************************** GUI ****************************/

// Rotate vector src around the origin by this many degrees
inline vec2 rotate(const vec2 &src,float ang_deg) {
	double ang_rad=ang_deg*M_PI/180.0;
	double s=sin(ang_rad), c=cos(ang_rad);
	return vec2( c*src.x-s*src.y, s*src.x+c*src.y);
}

inline float state_to_Y(int state) {
	return 0+field_y_size*(state_last-state)*(1.0/state_last);
}

/* Called at start of user's OpenGL display function */
void robot_display_setup(const robot_current &robot) {
	
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_ALPHA_TEST);
	glDisable(GL_BLEND);
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	int wid=glutGet(GLUT_WINDOW_WIDTH), ht=glutGet(GLUT_WINDOW_HEIGHT);
	glViewport(0,0,wid,ht);
	
	// Encode current robot state in background color:
	if (robot.state==state_STOP) {
		glClearColor(0.7,0.0,0.0,0.0); // bright red (STOP sign)
	}
	else if (robot.state==state_drive) {
		glClearColor(0.3,0.3,0.3,0.0); // drive: dim gray
	}
	else {
		glClearColor(0.1,0.2,0.4,0.0); // blue autonomy
	}

	glClear(GL_COLOR_BUFFER_BIT+GL_DEPTH_BUFFER_BIT);
	
	// Scale to showing the whole field, in centimeter units
	float xShift=-0.5, yShift=-0.9;
	glTranslatef(xShift,yShift,0.0);
	float yScale=1.8/field_y_size;
	float xScale=yScale*ht/wid;
	glScalef(xScale, yScale, 0.1);
	robotPrintf_y=(1.0-yShift)/yScale+robotPrintf_line;
	
	// Read back the matrix to get from cm to onscreen pixels
	float mat[16];
	glGetFloatv(GL_MODELVIEW_MATRIX,mat);
	int w=glutGet(GLUT_WINDOW_WIDTH), h=glutGet(GLUT_WINDOW_HEIGHT);
	vec2 mat_scale(1.0/mat[0],1.0/mat[5]);
	vec2 mat_offset(mat[12],mat[13]);
	
	// coordinate-convert mouse to cm coords
	vec2 m=vec2(robotMouse_pixel.x*2.0/w-1.0,(h-robotMouse_pixel.y)*2.0/h-1.0)-mat_offset;
	m.x*=mat_scale.x;
	m.y*=mat_scale.y;
	robotMouse_cm=m;
	
	glLineWidth(1+3*wid/1000);
	
	/*
	glBegin(GL_LINES); // to verify mouse position
	glColor4f(0.0,0.6,0.0,1.0);
	glVertex2fv(robotMouse_cm);
	glVertex2fv(robotMouse_cm+vec2(10,20));
	glEnd();
	*/

// Delineate the start and mine zones
	glBegin(GL_LINES);
	glColor4f(0.3,0.3,0.5,1.0);
	glVertex2i(-field_x_hsize,field_y_start_zone);
	glVertex2i(+field_x_hsize,field_y_start_zone);
	glVertex2i(-field_x_hsize,field_y_mine_zone);
	glVertex2i(+field_x_hsize,field_y_mine_zone);

// Draw the lunabin
	glColor4f(0.3,1.0,1.0,1.0);
	glVertex2i(-field_x_hbin,-10);
	glVertex2i(+field_x_hbin,-10);
	glEnd();
	
// Outline the field
	glBegin(GL_LINE_LOOP);
	glColor4f(0.0,0.0,0.8,1.0);
	glVertex2i(+field_x_hsize,0);
	glVertex2i(-field_x_hsize,0);
	glVertex2i(-field_x_hsize,field_y_size);
	glVertex2i(+field_x_hsize,field_y_size);
	glEnd();

// Draw the current autonomy state
	robotPrintf_enable=false;
	double state_display_x=3*field_x_hsize;
	for (robot_state_t state=state_STOP;state<state_last;state=(robot_state_t)(state+1))
	{
		glColor4f(0.0,0.0,0.0,1.0); // black inactive
		
		if (state==robotState_requested || (
		    robotMouse_cm.x>state_display_x && 
		    robotMouse_cm.y<state_to_Y(state) && 
		    robotMouse_cm.y>state_to_Y(state+1)
		    ))
		{ // red mouse hover
			glColor4f(1.0,0.0,0.0,1.0);
			if (robotMouse_down==true)
			{ // request new state
				robotState_requested=state;
			}
		}
		
		if (state==robot.state) {
			glColor4f(1.0,1.0,1.0,1.0); // white when active
		}
		robotPrint(state_display_x,
			0.5*(state_to_Y(state)+state_to_Y(state+1)), // average height
			state_to_string(state));
	}
	robotPrintf_enable=true;
	
// Draw current robot power values
	unsigned char *powers=(unsigned char *)&robot.power; // HACK: want array of powers
	glBegin(GL_TRIANGLES);
	for (unsigned int i=0;i<sizeof(robot.power);i++) {
		unsigned int pow=powers[i]&0x7f;
		int autonomous=powers[i]&0x80;
		float cenx=50*(0.5+i)+field_x_GUI;
		float ceny=0.10*field_y_size;
		glColor3ub(128+pow,autonomous?255:128,255-pow);
		glVertex2f(cenx-20,ceny);
		glVertex2f(cenx+20,ceny);
		glVertex2f(cenx,ceny+2.0*(powers[i]-60));
	}
	glEnd();

// Draw current robot configuration (side view)
	glBegin(GL_TRIANGLES);
	double robot_draw_y=75; // size of side view image
	double robot_draw_x=75;
	vec2 robot_draw(0.6*field_x_size,200);
	vec2 dump_pivot=robot_draw+vec2(0,robot_draw_y);
	vec2 front_pivot=robot_draw+vec2(robot_draw_x,0);
	
	
	glColor4f(0.0,0.0,0.0,1.0); // body (black)
	glVertex2fv(robot_draw);
	glVertex2fv(dump_pivot);
	glVertex2fv(front_pivot);
	
	double front_angle=100.0*((robot.sensor.frontL-50.0)/620.0);
	glVertex2fv(front_pivot);
	glVertex2fv(front_pivot + rotate(vec2(robot_draw_x,20),front_angle));
	glColor4f(1.0,0.0,0.0,0.5); // front wheel tip (red)
	glVertex2fv(front_pivot + rotate(vec2(robot_draw_x,0),front_angle));
	
	glColor4f(0.0,0.0,0.0,1.0); // body (black)
	double dump_angle=150.0*((robot.sensor.bucket-100.0)/800.0)-50.0;
	glVertex2fv(dump_pivot);
	glVertex2fv(dump_pivot + rotate(vec2(robot_draw_x*1.3,0),dump_angle));
	glColor4f(0.0,1.0,0.0,0.5); // mining head (green)
	glVertex2fv(dump_pivot + rotate(vec2(robot_draw_x*1.3,-20),dump_angle));
	
	glEnd();
	
// Output telemetry as text (for log, mostly)
	glColor3f(1.0,1.0,1.0);
	
	char fillBuckit[] = "_ _ _ _ _ _";
	for(int i=0; i<6; i++)
		if(robot.sensor.bucketFill & (1<<i))
			fillBuckit[2*i] = '1';
	robotPrintln("Bucket fill: %s  (moving %d, stall avg %d)",fillBuckit,robot.sensor.mineMoving, robot.sensor.avgStall);
	
	if (robot.status.arduino) 
	{ // arduino connected: print status
		std::string status="";
		if (robot.sensor.backL) status+="LEFTBACK ";
		if (robot.sensor.backR) status+="    RIGHTBACK ";
		if (robot.status.stop) status+="STOP(status) ";
		if (robot.sensor.stop) status+="STOP(sensor) ";
		if (robot.status.located) status+="located ";
		if (robot.status.autonomy) status+="AUTONOMY ";
		if (robot.status.semiauto) status+="SEMIAUTO ";
		robotPrintln("Arduino connected: %s",status.c_str());
		
	// Analog voltage dividers:
	// Linear actuators:
		robotPrintln("  bucket %.1f%% (%d) up",
			robot.sensor.bucket*100.0/700.0,robot.sensor.bucket);
		robotPrintln("  frontL %.1f%% (%d) out",
			robot.sensor.frontL*100.0/1023.0,robot.sensor.frontL);
		robotPrintln("  frontR %.1f%% (%d) out",
			robot.sensor.frontR*100.0/1023.0,robot.sensor.frontR);
		
		robotPrintln("  battery %.2f V (%d)",
			robot.sensor.battery*AD_DN2high_voltage,robot.sensor.battery);
		
		robotPrintln("  mining %.2f V (%d, del %d)",
			robot.sensor.mining*AD_DN2high_voltage,robot.sensor.mining,
			robot.sensor.battery-robot.sensor.mining);
		if (robot.power.mine!=64 && robot.sensor.stalled) robotPrintln("    (mining head stalled)");

		robotPrintln("  latency %d",
			robot.sensor.latency);
	} else {
		robotPrintln("Arduino not connected");
	}
	
	if (robot.loc.confidence>0.5) {
		robotPrintln("Location: %.1f, %.1f, %1f  angle %.1f",
			robot.loc.x,robot.loc.y,robot.loc.z, 
			robot.loc.angle);
	}
}

void robot_display(const robot_localization &loc,double alpha=1.0)
{
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	
// Draw the robot
	float conf=loc.confidence;
	glColor4f(0.8,0.8*conf,0.8*conf,alpha);
	glBegin(GL_TRIANGLE_FAN);
	float ang=loc.angle*M_PI/180.0;
	vec2 C(loc.x,loc.y); // center of robot
	vec2 F(+60.0*sin(ang), +60.0*cos(ang)); // robot forward direction
	vec2 R(+70.0*cos(ang), -70.0*sin(ang)); // robot right side
	double d=1.0; // front wheel deploy?
	
	glColor4f(0.0,0.8*conf,0.0,alpha); // green center
	glVertex2fv(C+0.5*F);
	
	glColor4f(0.8*conf,0.0,0.0,alpha); // red front wheels
	glVertex2fv(C-R+d*F);
	
	glColor4f(0.0,0.0,0.0,alpha); // black back
	glVertex2fv(C-R-F);
	glVertex2fv(C+R-F);
	
	glColor4f(0.8*conf,0.0,0.0,alpha); // red front wheels
	glVertex2fv(C+R+F);
	glEnd();
	
	glColor4f(1.0,1.0,1.0,1.0);
}


/*************************** Keyboard **********************************/
/** Handle keyboard presses */
#include "../ogl/event.h"

extern "C" void ogl_main_keyboard(unsigned char key, int x, int y)
{
        // flip toggles on keypress
        oglToggles[key]=!oglToggles[key];
        oglKeyMap[key]=1;
}
extern "C" void ogl_main_keyboard_up(unsigned char key, int x, int y)
{
        oglKeyMap[key]=0;
}
extern "C" void ogl_main_special(int key, int x, int y)
{
        if (key<0x80)
                oglKeyMap[0x80+key]=1;
}
extern "C" void ogl_main_special_up(int key, int x, int y)
{
        if (key<0x80)
                oglKeyMap[0x80+key]=0;
}

void ogl_mouse_motion(int x, int y) { 
	robotMouse_pixel=vec2(x,y);
}

void ogl_mouse(int button,int state,int x,int y) 
{ /* mouse being pressed or released--save position for motion */
	ogl_mouse_motion(x,y);
	if (state==GLUT_DOWN) {
	       robotMouse_down=true;
	} else {
	       robotMouse_down=false;
	}
}

void robotMainSetup(void) {
	glutKeyboardFunc (ogl_main_keyboard);
	glutKeyboardUpFunc (ogl_main_keyboard_up); /* "up" version for KeyMap */
	glutSpecialFunc (ogl_main_special); /* for arrow keys */
	glutSpecialUpFunc (ogl_main_special_up);
	glutMouseFunc(ogl_mouse);
	glutMotionFunc(ogl_mouse_motion); 
	glutPassiveMotionFunc(ogl_mouse_motion); 

}



#endif


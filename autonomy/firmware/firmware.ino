/**
  Aurora Robotics "Autonomy" firmware
  For Arduino Mega
*/
#include "robot.h" /* classes shared with PC, and across network */
#include "serial_packet.h" /* CYBER-Alaska packetized serial comms */

// All PC commands go via this (onboard USB) port
HardwareSerial &PCport=Serial; // direct PC
HardwareSerial &PCport2=Serial2; //donor's RX/TX2

// Call this function frequently--it's for minimum-latency operations
void low_latency_ops();

/** This class manages communication via an A_packet_formatter,
    including timeouts. */
class CommunicationChannel {
public:
  HardwareSerial &backend;
  A_packet_formatter<HardwareSerial> pkt; // packet formatter
  bool is_connected; // 1 if we're recently connected; 0 if no response
  unsigned long last_read; // millis() the last time we got data back
  unsigned long next_send; // millis() the next time we should send off data
  
  CommunicationChannel(HardwareSerial &new_backend) :backend(new_backend), pkt(backend) 
  {
    is_connected=0;
    last_read=0;
    next_send=0;
  }
  
  bool read_packet(unsigned long milli, A_packet &p) {
     p.valid=0;
     if (backend.available()) {
       while (-1==pkt.read_packet(p)) { 
         low_latency_ops(); /* while reading packet */
       }
       if (p.valid) {
  	 last_read=milli; next_send=milli+500;
  	 is_connected=true; // got valid packet
         return true;
       }
    }
    if (milli>next_send) { // read timeout
  	next_send=milli+500;
  	pkt.reset();
  	pkt.write_packet(0,0,0); // send heartbeat ping packet
	is_connected=false;
	digitalWrite(13,LOW);
    }
    return false;
  }
};
CommunicationChannel PC(PCport);
CommunicationChannel PCdonor(PCport2);

/*************** Sabertooth 2x25 v2.00 *******************/
// All sabertooth commands leave via this port
//   (Compile error here?  Tools -> Board -> Arduino Mega 2560)
HardwareSerial &SABERport=Serial1;//TX1

// These MUST match the sabertooth DIP address pins!
const int leftAddr=128;
const int rightAddr=128;
const int frontAddr=129;
const int mineAddr=130;

const int motor1=7; // command motor 1
const int motor2=6; // command motor 2 (note: backwards of documentation!)

/**
 Set this sabertooth to this motor power level.
    addr = sabertooth being commanded
    motor = motor1 or motor2
    val = 1 (reverse) 64 (stop) 127 (forward) power

 At 9600 baud, this function takes 4 milliseconds (unless maybe it's buffered?)
*/
void sendMotor(int addr, int motor, int val){
  unsigned char chksum=(addr+motor+val)&127;
  unsigned char pkt[4];
  pkt[0]=addr; pkt[1]=motor; pkt[2]=val; pkt[3]=chksum;
  SABERport.write(pkt,4);
/*
  SABERport.write(addr);
  SABERport.write(motor);
  SABERport.write(val);
  SABERport.write(chksum);
*/
}

/***************** Robot Control Logic ****************/
int pin_bumper_L=44;
int pin_bumper_R=47;
int pin_voltage_supply=49;
int front_encoder_power=52;

long mine_last=0; // millis() at last mining motion check
int bucket_last=0; // bucketFill at last mining motion check
int bucket_last2=0; // last-last

long stall_last=0; // timer for stall detection average
int sum_stalled=0, sum_stall_count=0;

void setup()
{
  Serial.begin(9600); // Control connection to PC
  Serial2.begin(9600); // Control connection to PC
  
  SABERport.begin(9600); // <- maybe try 38400 for faster updates? (set switches)
  delay(2000); // Sabertooth docs: "allow a two-second delay between applying power and..."
  SABERport.write(0xAA); // "first character must be 170 (AA in hex)"
  delay(10); // <- needed?

 // Our ONE debug LED!
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);

 // Pin modes for inputs
  pinMode(pin_bumper_L,INPUT_PULLUP);
  pinMode(pin_bumper_R,INPUT_PULLUP);
  pinMode(pin_voltage_supply,OUTPUT);
  pinMode(front_encoder_power,OUTPUT);
  digitalWrite(pin_voltage_supply,HIGH);
  digitalWrite(front_encoder_power,HIGH);
  
/*
  // buckit fill sensors
  for(int i=22; i<=32; i+=2)
    pinMode(i, INPUT);
  pinMode(34, OUTPUT);
  digitalWrite(34, LOW);
  */
  
  // Infrared detectors
  for (int pin=2;pin<=7;pin++) 
  {
    digitalWrite(pin,LOW);
    pinMode(pin,OUTPUT); 
  }
  pinMode(A4,OUTPUT); // A4 is an LED output pin
  digitalWrite(A4,LOW);
}

// Robot's current state:
robot_current robot;

// Read all robot sensors into robot.sensor
void read_sensors(void) {
#if 0 /* This is for the old two-RJ board */
	robot.sensor.battery=analogRead(A10); // 24V bus
	low_latency_ops();
	robot.sensor.mining=analogRead(A12); // mining bus
	low_latency_ops();
	robot.sensor.bucket=analogRead(A0); // linear actuator (TEMP)
	low_latency_ops();
	robot.sensor.frontL=analogRead(A14);
	low_latency_ops();
	robot.sensor.frontR=analogRead(A15);
#else /* This is the new 4-RJ shield */
/*
  Power RJ:
  A8: orange, unused
  A9: blue
  A10: green mining voltage
  A11: brown 24v bus
  A12: white/brown, main ground
  A13: bucket green
  A14: front blue?
  A15: front green?
*/
	robot.sensor.mining=analogRead(A10); // mining (for stall)
	low_latency_ops();
	robot.sensor.battery=analogRead(A11); // 24V bus
	low_latency_ops();
	robot.sensor.bucket=analogRead(A13); // linear actuator (TEMP)
	low_latency_ops();
	robot.sensor.frontL=analogRead(A14);
	low_latency_ops();
	robot.sensor.frontR=analogRead(A15);
#endif

#if 0 /* old bucket fill sensors */
        robot.sensor.bucketFill = 0;
        
        int pinMap[6] = {24, 22, 28, 26, 30, 32};
        for(int i=0; i<6; i++) {
          if(digitalRead(pinMap[i]))
            robot.sensor.bucketFill |= (1<<i);
        }
#else  /* New infrared sensors */
  /*     Order: orange, green, blue: mining, full, half          */
  static const int ANApins[6]={A5,A6,A7, A1,A2,A3}; 
  static const int LEDpins[6]={2,3,4,    6,A4,7};
  robot.sensor.bucketFill = 0;
  
  digitalWrite(5,HIGH); // orange, 5V feed for all IR detectors
  for (int i=0;i<6;i++) {
    digitalWrite(LEDpins[i],HIGH);
    int onBright=analogRead(ANApins[i]);
    low_latency_ops();
    
    digitalWrite(LEDpins[i],LOW);
    int offBright=analogRead(ANApins[i]);
    low_latency_ops();
    
    if (onBright>offBright+50) 
      robot.sensor.bucketFill |= (1<<i);
  }
  digitalWrite(5,LOW); // all detectors off
  
  // mine stall detection state updates
  long cur_millis=millis();
  if (cur_millis>=mine_last+10) { // 10ms isochronous run
    mine_last=cur_millis;
    
    sum_stalled+=robot.sensor.battery-robot.sensor.mining;
    sum_stall_count++;
    if (cur_millis>=stall_last+100) 
    {
      if (sum_stall_count!=0)
      robot.sensor.avgStall=sum_stalled/sum_stall_count;
      sum_stalled=sum_stall_count=0;
      stall_last=cur_millis;
    }
    
    
    int moving=robot.sensor.mineMoving;
    if ((bucket_last & bucket_last2 &1) != (robot.sensor.bucketFill&1)) {
      moving+=50; // transition? definitely turning 
    } else {
      moving-=10; // no transition? might not be turning
    }
    if (moving<0) moving=0;
    if (moving>200) moving=200;
    robot.sensor.mineMoving=moving;
    
    bucket_last2=bucket_last;
    bucket_last=robot.sensor.bucketFill;
  }
  
  
#endif


	robot.sensor.stop=robot.sensor.battery<600;
	robot.sensor.backL=!digitalRead(pin_bumper_L);
	robot.sensor.backR=!digitalRead(pin_bumper_R);
	robot.sensor.stalled=(robot.sensor.battery-robot.sensor.mining)>30;
	if (robot.power.mineMode) {
	  // AUTONOMOUS RESPONSE: if stalled, alternate between raising bucket and driving forward
	  if (robot.sensor.stalled) 
	  {
	        static unsigned int count=0;
                if(count%2==0) 
		{ // raise bin
		     robot.power.dump=127; 
		     count++;
		}
	        else 
		{ // drive backward
		     robot.power.left=robot.power.right=54; // should correspond to about 10% of total power(trial and error required)
		     count++;
		} 
	  } 
	  else { // not stalled
                robot.power.dump=50; // weakly lower bucket
                robot.power.left=robot.power.right=64+(0.2*64); // drive forward
          }
	}
	
	if(robot.power.backMode)
	{
		const int back_slow=52; // <- raw 7-bit power value for slow back-up
		if (robot.sensor.backL) { robot.power.left=robot_power::drive_stop; } 
		else { robot.power.left=back_slow; }
		if (robot.sensor.backR) { robot.power.right=robot_power::drive_stop; } 
		else { robot.power.right=back_slow; }
		
		if (robot.sensor.backL && robot.sensor.backR) 
		{ // enter dump mode
			robot.power.backMode=0;
			robot.power.dumpMode=1;
			robot.power.dump=127;
		}
	}
}

// Send current motor values to sabertooths.
//   These 5 writes take a total of 20ms *if* the serial line is busy
void send_motors(void) {
	sendMotor(leftAddr,motor1,robot.power.left);
	sendMotor(rightAddr,motor2,128-robot.power.right);
	low_latency_ops();

	sendMotor(frontAddr,motor1,robot.power.front);
	low_latency_ops();

	sendMotor(mineAddr,motor1,robot.power.mine);
	sendMotor(mineAddr,motor2,robot.power.dump);
}

// Structured communication with PC:
void handle_packet(A_packet_formatter<HardwareSerial> &pkt,const A_packet &p)
{
	if (p.command==0x7) { // motor power commands
		low_latency_ops();
		if (!p.get(robot.power)) { // error
			pkt.write_packet(0xE,0,0);
		}
		else 
		{ // got power request successfully: read and send sensors
			low_latency_ops(); /* while reading sensors */
			read_sensors();
			low_latency_ops();
			pkt.write_packet(0x3,sizeof(robot.sensor),&robot.sensor);
			robot.sensor.latency=0; // reset latency metric
			low_latency_ops();
			
			static bool blink=0;
			digitalWrite(13,!blink); // good input received: blink!
			blink=!blink;
		}
  	}
  	else if (p.command==0) { // ping request
		pkt.write_packet(0,p.length,p.data); // ping reply
	}
}

unsigned long next_micro_send=0;
void loop()
{
  unsigned long micro=micros();
  unsigned long milli=micro>>10; // approximately == milliseconds
  
  A_packet p;
  if (PC.read_packet(milli,p)) handle_packet(PC.pkt,p);
  if (PCdonor.read_packet(milli,p)) handle_packet(PCdonor.pkt,p);
  if (!(PC.is_connected||PCdonor.is_connected)) robot.power.stop(); // disconnected?
  
  if (micro>=next_micro_send) 
  { // Send commands to motors
    send_motors();
    next_micro_send=micro+25*1024; // send_motors takes 20ms
  }
  
  low_latency_ops();
}

/**** Low latency (sub-millisecond) timing section.
   We need this for maximum accuracy in our blinky readings.
*/
robot_blinky_report blinky_report;
unsigned int blinky_index=0;
unsigned int blinky_sync=0;

unsigned long last_micro_loop=0;
void low_latency_ops() {
  unsigned long micro=micros();
  unsigned int cycle=micro%1024u;
  const unsigned int cycle_end=800u; // < 1024 - maximum latency
#if 0 /* no need for blinky */
  if ((PC.is_connected || PCdonor.is_connected) && cycle>cycle_end) 
  { /* wait for timer tickover, then take sample */
    while (((micro=micros())%1024u)>cycle_end) { /* busywait */ }
    unsigned int blinky_value=analogRead(A7);
    micro=last_micro_loop=micros(); // ignore above for latency calculation
    
    digitalWrite(36, blinky_sync);// swap LED, for next reading
    blinky_sync=!blinky_sync;
    
    blinky_report.write(blinky_index,blinky_value);
    blinky_index++; 
    if (blinky_index>=robot_blinky_report::n_report) {
      blinky_index=0;
      blinky_report.sync=blinky_sync;
      blinky_report.timing=micro/1024;
      PC.pkt.write_packet(0xB,sizeof(blinky_report),&blinky_report);
      PCdonor.pkt.write_packet(0xB,sizeof(blinky_report),&blinky_report);
    }
  }
#endif

  // Update latency counter
  unsigned int latency=micro-last_micro_loop;
  // latency/=1024; // milliseconds (ish)
  if (latency>=1024) latency=1023;
  if (robot.sensor.latency<latency) robot.sensor.latency=latency;
  last_micro_loop=micro;
}



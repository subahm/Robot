#pragma config(Sensor, in1,    IR1,            sensorAnalog)
#pragma config(Sensor, in2,    IR2,            sensorAnalog)
#pragma config(Sensor, dgtl1,  signal1,        sensorDigitalOut)
#pragma config(Sensor, dgtl2,  signal2,        sensorDigitalOut)
#pragma config(Sensor, dgtl3,  rangeFinders,   sensorDigitalOut)
#pragma config(Sensor, dgtl4,  button1,        sensorTouch)
#pragma config(Sensor, dgtl5,  button2,        sensorTouch)
#pragma config(Sensor, dgtl6,  sonar,          sensorSONAR_cm)
#pragma config(Motor,  port1,           motorArm,      tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           motorL,        tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port3,           motorR,        tmotorServoContinuousRotation, openLoop, reversed)

//Written by Subah Mehrotra, Zhenhan Zhang, and Gregory O'Hagan
//Lab section B14, group 056

//initiallizes global variables for the two IR sensors and two push buttons
//this avoids passing numerous variables to and from each method

bool onePressed;
bool twoPressed;

int IR1Min;
int IR1Max;
int IR2Min;
int IR2Max;

//global time for 1 rotation with the motors at speed +/- 20, allows for easy callibrating
//All methods that use a full circle (typically to seach for the best input) use this number
int circleTime = 3900;

//Simple method for callibrating the time to complete one circle
//Calling this method causes the robot to attempt to do one complete circle
//If this turns too far or not far enough, adjusting the time global variable above
//corrects this for the entire program
void completeCircle(){
	motor[motorL] = 20;
	motor[motorR] = -20;
	wait1Msec(circleTime);
	motor[motorL] = 0;
	motor[motorR] = 0;
}

//Continuously running this method checks for button inputs
void monitorInput(){
	if(SensorValue(button1) && !onePressed){
		onePressed = true;
	}
	if(SensorValue(button2) && !twoPressed){
		twoPressed = true;
	}
}

//Detects the minimum and maximum values of the IR sensors over 1/20 of a second
//A significant difference between these detects the beacon (10Hz IR signal)
//When using the IR detectors as rangefinders, using the minimum value avoids interference from the beacon
void monitorIR(){
	IR1Min = SensorValue(IR1);
	IR1Max = SensorValue(IR1);
	IR2Min = SensorValue(IR2);
	IR2Max = SensorValue(IR2);
	wait1Msec(50);
	if (SensorValue(IR1) > IR1Max){
		IR1Max = SensorValue(IR1);
	}
	else{
		IR1Min = SensorValue(IR1);
	}
	if (SensorValue(IR2) > IR2Max){
		IR2Max = SensorValue(IR2);
	}
	else{
		IR2Min = SensorValue(IR2);
	}
}

//Points the robot towards the target
//Does this by performing a 360 degree scan, then faces towards the strongest beacon
void findTarget(){
	int IRDiffMax = 0;
	int timeOfMin = 0;
	clearTimer(T1);
	motor[motorL] = 20;
	motor[motorR] = -20;
	//loop detects the greatest difference between IR high and low readings as the robot spins
	//finding the greatest difference prevents the robot from following reflections
	while (time1[T1] < circleTime){
		monitorIR();
		if (IR1Max - IR1Min > IRDiffMax){
			IRDiffMax = IR1Max - IR1Min;
			timeOfMin = time1[T1];
		}
	}
	//turns to the angle detected above
	//-100 is added to account for the robot not stopping and starting again
	while (time1[T1] < circleTime + timeOfMin - 100){
		wait1Msec(1);
	}
	motor[motorL] = 0;
	motor[motorR] = 0;
	wait1Msec(200);
}

//Handles initial target approach. Gets the robot within 60 cm of the target and facing the target
void approachTarget(){
	motor[motorL] = 50;
	motor[motorR] = 50;
	//checks the two IR sensor differences (the strength of the 10Hz signal each is receiving)
	//adjusts course based on these to home in on the target
	while (SensorValue(sonar) > 100){
		if (IR1Max - IR1Min > IR2Max - IR2Min + 50){
			motor[motorL] = 50;
			motor[motorR] = 40;
		}
		else if (IR2Max - IR2Min > IR1Max - IR1Min + 50){
			motor[motorL] = 40;
			motor[motorR] = 50;
		}
		else {
			motor[motorL] = 50;
			motor[motorR] = 50;
		}
	}
	//Same logic as above, but with lower speeds and bigger adjustments
	//This keeps the initial approach fast while avoiding overshooting as it gets close
	while (SensorValue(sonar) > 60){
		monitorIR();
		if (IR1Max - IR1Min > IR2Max - IR2Min + 50){
			motor[motorL] = 30;
			motor[motorR] = 20;
		}
		else if (IR2Max - IR2Min > IR1Max - IR1Min + 50){
			motor[motorL] = 20;
			motor[motorR] = 30;
		}
		else {
			motor[motorL] = 30;
			motor[motorR] = 30;
		}
		//failsafe: if it loses the target signal, returns the robot to the "target search" state
		//The double check prevents momentary sensor glitches from triggering the failsafe, and is
		//standard in all checks that reset to a previous state in this program
		if (IR1Max - IR1Min < 100 && IR2Max - IR1Min < 100){
			wait1Msec(100);
			monitorIR();
			if (IR1Max - IR1Min < 100 && IR2Max - IR1Min < 100){
				findTarget();
			}
		}
	}
	motor[motorL] = 0;
	motor[motorR] = 0;
	wait1Msec(200);
}

//When initially facing the target, lines the robot up perpendicular to the target
//at the right distance for a drop
void  finalTargetApproach(){
	motor[motorL] = 20;
	motor[motorR] = 20;
	bool changed = false;
	//Steers the robot perpendicular to the target. Sonar detects the right distance for a drop
	while (SensorValue(sonar) > 9){
		changed = false;
		monitorIR();
		if (IR1Max -IR1Min > IR2Max - IR2Min + 100){
			motor[motorL] = 25;
			motor[motorR] = 20;
			changed = true;
		}
		if (IR2Max - IR2Min > IR1Max - IR2Min + 100){
			motor[motorL] = 20;
			motor[motorR] = 25;
			changed = true;
		}
		if (!changed){
			motor[motorL] = 20;
			motor[motorR] = 20;
		}
	}
	motor[motorR] = 0;
	motor[motorL] = 0;
	//If both IR sensors lose track of the puck, the robot backs up
	//(to avoid hitting the puck while turning if too close) and returns to the findTarget state
	if (IR2Max - IR2Min < 10 && IR2Max - IR2Min < 10){
		wait1Msec(100);
		if (IR2Max - IR2Min < 10 && IR2Max - IR2Min < 10){
			motor[motorL] = -30;
			motor[motorR] = -30;
			wait1Msec(1000);
			motor[motorL] = 0;
			motor[motorR] = 0;
			findTarget();
			approachTarget();
			finalTargetApproach();
		}
	}
}

//drops the object, then backs up and retracts the arm
void dropObject(){
	clearTimer(T1);
	//deploys arm. Slows down towards end of movement for a smoother drop
	motor[motorArm] = 30;
	while (time1[T1] < 1000){
	}
	motor[motorArm] = 15;
	while (time1[T1] < 1200){
	}
	motor[motorArm] = 0;
  wait1Msec(100);
  //Backs away from target (releasing object in the process)
	motor[motorL] = -30;
	motor[motorR] = -30;
	while (time1[T1] < 2300){
      wait1Msec(1);
	}
	motor[motorL] = 0;
	motor[motorR] = 0;
	//Retracts arm
	motor[motorArm] = -30;
	while (time1[T1] < 3400){
	  wait1Msec(1);
	}
	motor[motorArm] = 0;
	wait1Msec(100);
}

//Finds the closest wall behind the robot, then approaches it (rear 180 degrees since puck is expected to be in front)
//Uses an IR check to avoid the puck if this method is called from an unexpected position
//(not immediately after a delivery)
void findWall(){
	int distanceMin = 1000;
	int timeOfMin = 0;
	bool foundPuck = false;
	motor[motorL] = 20;
	motor[motorR] = -20;
	//Finds the nearest wall within 180 degree arc away from puck
	clearTimer(T1);
	wait1Msec(circleTime / 4);
	while (time1[T1] < 3 * circleTime / 4){
		//checks to make sure it isn't looking at the target
		monitorIR();
		if (IR1Max + IR2Max - IR1Min - IR2Min > 200){
			foundPuck = true;
		}
		else {
			foundPuck = false;
		}
		//Searches for the nearest object not emitting IR signals
		if (SensorValue(sonar) < distanceMin && SensorValue(sonar) > 5 && !foundPuck){
			distanceMin = SensorValue(sonar);
			timeOfMin = time1[T1];
		}
	}
	//Turns back to the position that gave the smallest distance reading
	while (time1[T1] < circleTime + timeOfMin - 100){
		wait1Msec(1);
	}
	motor[motorL] = 0;
	motor[motorR] = 0;
	wait1Msec(200);
	//approaches wall: speed is greater when further from wall
	while (SensorValue(sonar) > 30){
		motor[motorL] = 30;
		motor[motorR] = 30;
	}
	while (SensorValue(sonar) > 15){
		motor[motorL] = 18;
		motor[motorR] = 18;
	}
	motor[motorL] = 0;
	motor[motorR] = 0;
	wait1Msec(200);
}

//Lines the robot up perpendicular to a wall, then closes to within 5cm
void finalWallApproach(){
	monitorIR();
	//check to ensure that it is approaching a wall, not the beacon
	if (IR1Max - IR1Min > 500 || IR2Max - IR2Min > 500){
		findWall();
	}
	//Lines the robot up perpendicular to the wall using the front two IR sensor/LED pairs
	monitorIR();
	while (IR1Min > IR2Min + 20){
	 motor[motorL] = 18;
	 motor[motorR] = -18;
	 monitorIR();
	}
	motor[motorL] = 0;
	motor[motorR] = 0;
	while (IR2Min > IR1Min + 20){
	 motor[motorR] = 18;
	 motor[motorL] = -18;
	 monitorIR();
	}
	motor[motorR] = 0;
	motor[motorL] = 0;
	//Once lined up properly, closes to within the 5cm target
	while (SensorValue(sonar) > 5){
		motor[motorR] = 18;
		motor[motorL] = 18;
	}
	motor[motorR] = 0;
	motor[motorL] = 0;
}

//Lights up both LEDs for 5 seconds
void signalCompletion(){
	SensorValue(signal1) = 0;
	SensorValue(signal2) = 0;
	wait1Msec(5000);
	SensorValue(signal1) = 1;
	SensorValue(signal2) = 1;
}

//Debugging/calibrating method to manually move the arm without mechanically detaching it
//Not used in final program (also stays running once called until the robot is turned off)
//Holding button 1 moves the arm forward, holding button 2 retracts the arm
void moveArm(){
	while (true){
		monitorInput();
		if (onePressed){
			motor[motorArm] = 20;
			onePressed = false;
		}
		else if (twoPressed){
			motor[motorArm] = -20;
			twoPressed = false;
		}
		else{
			motor[motorArm] = 0;
		}
	}
}

//Runs all methods in their standard order
//Note that if a failsafe is triggered and an earlier method is called, that is handed by the method in which
//the failsafe was triggered, not this one.
void stateFlow(){
	findTarget(); //Points the robot towards the target
	approachTarget(); //Approaches the target to within 60 cm
	finalTargetApproach(); //Lines up the robot with the target and approaches to about 8 cm (drop distance)
	dropObject(); //Delivers the object, then backs away from the target
	findWall(); //Points the Robot towards the closest wall behind the robot, then drives towards it
	finalWallApproach(); //Lines the robot up with the wall, then closes to within 5cm
	signalCompletion(); //Lights up both LEDs for 5 seconds
}

//Simple main method to easily run individual methods during debugging
//Continuously tracks sensor input until a button is pressed
//Pressing button 1 starts the main program
task main(){
	SensorValue(signal1) = 1;
	SensorValue(signal2) = 1;
	while(true){
		monitorIR();
		monitorInput();
		if(onePressed){
			stateFlow();
			onePressed = false;
		}
		if(twoPressed){
			completeCircle();
			twoPressed = false;
		}
	}
}

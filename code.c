#include "mindsensors-motormux.h"

//trivial function prototypes
void configure_all_sensors();
void drive(int aSpeed, int dSpeed);
void drive_both(int speed);
bool rotate(int aSpeed, int dSpeed, float rot_angle);

//non-trivial function prototypes
void track_color (int val);
void rotateBin(int brickColor);
void checkColor();
bool clean(const int ROBOT_SPEED, const int TURNING_SPEED, const int BELT_SPEED);
void final_output(int numTurns, int time);
bool endButton();

//global array used to contain the number of each brick colour
int colour_array[5] = {0,0,0,0,0};

//function used to configure all sensors
void configure_all_sensors()
{
	SensorType[S2] = sensorEV3_Gyro;
	wait1Msec(50);
	SensorMode[S2] = modeEV3Gyro_Calibration;
	wait1Msec(100);
	SensorMode[S2] = modeEV3Gyro_RateAndAngle;

	SensorType[S3] = sensorEV3_Ultrasonic;

	SensorType[S1] = sensorEV3_Color;
	wait1Msec(50);
	SensorMode[S1] = modeEV3Color_Color;

	SensorType[S4] = sensorI2CCustom;
	MSMMUXinit();
}

//powers both motors with the same power
void drive_both(int speed)
{
	motor[motorA] = -1 * speed;
	motor[motorD] = -1 * (speed+0.5);
}

//powers motors independently (for turning)
void drive(int aSpeed, int dSpeed) {
	motor[motorA] = -1* aSpeed;
	motor[motorD] = -1* (dSpeed+0.5);
}

//function used to rotate the robot a specific angle
bool rotate(int aSpeed, int dSpeed, float rot_angle)
{
	drive(aSpeed,dSpeed);
	while(abs(getGyroDegrees(S2)) < abs(rot_angle))
	{
		bool ended = endButton();
		if(ended == true)
		{
			return false;
		}
	}

	drive_both(0);
	return true;
}

//function used to add new bricks to the array, and
//display the current bricks on the EV3
void track_color (int val)
{
	colour_array[val - 2] = colour_array[val - 2] + 1;

	//printing to EV3 display
	displayString(4,"blue: %i", colour_array[0]);
	displayString(5,"green: %i", colour_array[1]);
	displayString(6,"yellow: %i", colour_array[2]);
	displayString(7,"red: %i", colour_array[3]);
	displayString(8,"white: %i", colour_array[4]);
}

//function used to rotate to the correct position
//and drop brick into it's correct spot
void rotateBin(int brickColor)
{
	/*
	COLOUR GUIDELINES: 
	
	0 = No Colour
	1 = Black
	2 = Blue
	3 = Green
	4 = Yellow
	5 = Red
	6 = White
	7 = Brown
	*/
	
	nMotorEncoder[motorB] = 0;

	//defining variables for distance needed to turn
	float distance = (brickColor-1) * (180/PI * 2);
	const int BIN_SPEED = 8;
	const int TRAPDOOR_SPEED = 5;

	motor[motorB] = BIN_SPEED;

	//turning the bin wheel to the correct spot
	while(abs(nMotorEncoder[motorB]) < distance)
	{}

	motor[motorB] = 0;
	wait1Msec(1000);

	//opening the trapdoor to release the brick
	MSMMotorEncoderReset(mmotor_S4_1);
	MSMMotor(mmotor_S4_1, TRAPDOOR_SPEED);
	wait1Msec(800);

	MSMMotor(mmotor_S4_1, 0);
	wait1Msec(3000);

	//closing the trapdoor
	MSMMotorEncoderReset(mmotor_S4_1);
	MSMMotor(mmotor_S4_1, (-1 * TRAPDOOR_SPEED));
	wait1Msec(800);

	//returning the wheel to its initial spot
	MSMMotor(mmotor_S4_1, 0);
	MSMMotorEncoderReset(mmotor_S4_1);
	motor[motorB] = -1 * BIN_SPEED;
	nMotorEncoder[motorB] = 0;

	while(abs(nMotorEncoder[motorB]) < distance)
	{}

	motor[motorB] = 0;
}

//function used to check the colour of a new brick
void checkColor()
{
	int color = SensorValue(S1);

	//checking to see when the colour detector changes
	if(color != 1 && color != 0)
	{
		//waiting to ensure colour is accurate
		wait1Msec(1000);
		if(color != 1 && color != 0)
		{
			//if the color is not apart of the wheel, move
			//to the yellow bin (4)
			if(color == 7)
			{
				color = 4;
			}
			color = SensorValue(S1);

			//add the brick to the array	
			track_color(color);

			//stops driving procedure
			drive_both(0);
			motor[motorC] = 0;
			
			//calls rotateBin to drop the brick
			rotateBin(color);
			
			//continues driving again
			drive_both(18);
			motor[motorC] = 25;
		}
	}
}

//function used to traverse around a room, sorting bricks when necessary
bool clean(const int ROBOT_SPEED, const int TURNING_SPEED, const int BELT_SPEED)
{
	nMotorEncoder[motorA] = 0;
	int numTurns = 0;
	int angle = 0;
	
	//defining an initial distance from the wall
	int distance = 30;

	//running the program for 13 turns, creating 3 rectangles
	while(numTurns < 13)
	{
//changing the distance from the wall depending on the number of turns
		if(numTurns > 8)
		{
			distance = 50;
		}
		else if(numTurns > 4)
		{
			distance = 40;
		}

		drive_both(ROBOT_SPEED);

		//checking to see if the kill switch is pressed
		bool ended = endButton();
		if(ended == true)
		{
			//exits the program if the end button is pressed
			return false;
	  	}

	  //checking the color to see if a brick has been taken
		checkColor();
		motor[motorC] = BELT_SPEED;

		//turning when a wall is seen
		if(SensorValue[S3] <= distance)
		{
			motor[motorC] = 0;
			rotate(TURNING_SPEED, -1 * TURNING_SPEED,angle);
			numTurns++;
			
			//adding to the angle to increase gyroscope accuracy
			angle+= 90;
		}
		motor[motorC] = BELT_SPEED;
	}
	//stops the driving and returns true to the main task
	motor[motorC] = 0;
	drive_both(0);
	return true;
}

//function used to output information to the EV3 Console
void final_output(int numTurns, int time)
{
	//using math to find the time in minutes and seconds
	int seconds = time/1000;
	int mins = seconds/60;
	int leftOverSeconds = seconds%60;

	//clear display
	eraseDisplay();
	displayString(1,"CLEANING FINISHED");
	int total_bricks = colour_array[0] + colour_array[1] +
 colour_array[2]+colour_array[3]+colour_array[4];
	
	//timer
	displayString(2,"Total Clean Up Time: %i:%i", mins, leftOverSeconds);
	
	//bricks
	displayString(4,"blue: %i", colour_array[0]);
	displayString(5,"green: %i", colour_array[1]);
	displayString(6,"yellow: %i", colour_array[2]);
	displayString(7,"red: %i", colour_array[3]);
	displayString(8,"white: %i", colour_array[4]);
	
	//total bricks
	displayString(9,"total: %i", total_bricks);
	
	//number of turns
	displayString(12, "total Turns: %i", numTurns);
}

//function used as a kill switch to end the program
bool endButton()
{
	//checking to see if a button has been pressed
	if(getButtonPress(buttonLeft) || getButtonPress(buttonRight) || 
   getButtonPress(buttonDown) || getButtonPress(buttonUp))
	{
//turns off all motors and returns true to signal that the program 
//should end
		motor[motorC] = 0;
		drive_both(0);
		MSMotorStop(mmotor_S4_1);
		MSMotorStop(mmotor_S4_2);
		return true;
	}
	//returns false to indicate no button has been pressed
	return false;
}

task main()
{
	//defining constants for speed
	const int ROBOT_SPEED = 15;
	const int BELT_SPEED = 15;
	const int TURNING_SPEED = 30;

	//creating value for timers
	time1[T1] = 0;
	int timer_val = 0;

	configure_all_sensors();
	MSMMotorEncoderReset(mmotor_S4_1);

	//waiting for the user to press the enter button
	while(!getButtonPress(buttonEnter))
	{
	}

	//waiting for the user to let go of the enter button
	while(getButtonPress(buttonEnter))
	{
	}

	//calls the function to drive around the area
	bool doneClean = clean(ROBOT_SPEED, TURNING_SPEED, BELT_SPEED);

	//after driving is done, checks to see if the driving was 
	//closed naturally, or by the kill switch
	if(doneClean == true)
	{
		//sends information to the final_output function
		timer_val = time1[T1];
		drive_both(0);
		final_output (4, timer_val);

		//waiting for the user to press the enter button
		while(!getButtonPress(buttonEnter))
		{
		}

		//waiting for the user to let go of the enter button
		while(getButtonPress(buttonEnter))
		{
		}

		//turns off all motors
		MSMMotorEncoderReset(mmotor_S4_1);
		MSMotorStop(mmotor_S4_1);
		MSMotorStop(mmotor_S4_2);
	}
}

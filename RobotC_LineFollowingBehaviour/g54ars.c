#pragma config(Sensor, S1,     gyro,           sensorEV3_Gyro)
#pragma config(Sensor, S2,     infaRight,      sensorEV3_Ultrasonic)
#pragma config(Sensor, S3,     HTCS2,          sensorI2CCustom)
#pragma config(Sensor, S4,     infaLeft,       sensorEV3_Ultrasonic)
#pragma config(Motor,  motorA,          rightMotor,    tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor,  motorC,          leftMotor,     tmotorEV3_Large, PIDControl, encoder)

#include "HitechnicColorSensor.h"

long r,g,b;

//Initiate variables for line following motor speed
bool followLine = false;
float followSpeed_L = 0;
float followSpeed_R = 0;

//Initiate variables for finding line motor speed
bool findLine = true;
float findSpeed_L = 10;
float findSpeed_R = 3;

//Initiate variables for avoidance motor speed
bool avoidLine = false;
float avoidSpeed_L = 0;
float avoidSpeed_R = 0;

//Initiating ultrasonic sensors and set object detection length
int usSensorLength = 6;
float sensor_R;
float sensor_L;

//Initiate variable to store average light value and setup threshold for line detection
int lightValue_Avg;
int threshold = 550;

void turnLeft(int x){
	clearTimer(T3);
	while(time1[T3] < 900 * x){
		avoidSpeed_L = -10;
		avoidSpeed_R = 10;
	}
}

void turnRight(int x){
	clearTimer(T4);
	while(time1[T4] < 900 * x){
		avoidSpeed_L = 10;
		avoidSpeed_R = -10;
	}
}

void stopRobot(){
	avoidSpeed_L = 0;
	avoidSpeed_R = 0;
}


void moveForward(){
	avoidSpeed_L = 15;
	avoidSpeed_R = 15;
}

void getDistance(){
	sensor_R = getUSDistance(infaRight);
	sensor_L = getUSDistance(infaLeft);
}

bool checkIfobst(){
	bool checkIfObstacle;
	getDistance();
	if (sensor_R < usSensorLength || sensor_L < usSensorLength){
		checkIfObstacle = true;
	}
	else{
		checkIfObstacle = false;
	}
	return checkIfObstacle;
}

void moveAroundObstacle(int x){
		bool obstacleThere = true;
		while (obstacleThere){
			clearTimer(T2);
			while(time1[T2] < 1500 * x){
				moveForward();
			}
			turnRight(2.6);
			obstacleThere = checkIfobst();
			turnLeft(2.6);
		}
		clearTimer(T2);
		while(time1[T2] < 1500 * x){
				moveForward();
		}
		turnRight(2.1);
		obstacleThere = true;
		clearTimer(T2);
		while(time1[T2] < 3000 * x){
				moveForward();
		}
		while (obstacleThere){
			clearTimer(T2);
			while(time1[T2] < 1000 * x){
				moveForward();
			}
			turnRight(2.3);
			obstacleThere = checkIfobst();
			turnLeft(2.3);
			}
			clearTimer(T2);
			while(time1[T2] < 2000 * x){
				moveForward();
			}
			turnRight(2.1);
			obstacleThere = checkIfobst();
			while(lightValue_Avg > threshold){
					moveForward();
		}
		turnLeft(1.1);
		avoidLine = false;
}

void getLightValues(){

	int lightValues_Array[5];

		for(int x = 0; x < 5; x++){
			HTCS2readRawRGB(HTCS2, true, r,g,b);
			lightValues_Array [x] = r;
		}
		lightValue_Avg = (lightValues_Array[0]+lightValues_Array[1]+
												lightValues_Array[2]+lightValues_Array[3]+
													lightValues_Array[4])/5;
}

task arbiterTask(){

	while(true){

		if(avoidLine){
			setMotorSpeed (leftMotor , avoidSpeed_L);
		   setMotorSpeed (rightMotor , avoidSpeed_R);
		}
		else{
			if (followLine){
				setMotorSpeed (leftMotor , followSpeed_L);
				setMotorSpeed (rightMotor , followSpeed_R);

			}
			else if(findLine){
				setMotorSpeed (leftMotor , findSpeed_L);
				setMotorSpeed (rightMotor , findSpeed_R);
			}
		}
	}
}

task avoidTask(){

	while(true){
		getDistance();
		if(checkIfobst()){

			avoidLine = true;
			stopRobot();
			wait1Msec(500);
			turnLeft(2.6);
			stopRobot();
			moveAroundObstacle(1);
		}
	}
}

//Line following behaviour based on recorded light levels
task followLineTask(){
     while(true){
        //Black detected - Turn Left
        if(lightValue_Avg < threshold){
            followLine = true;
            followSpeed_L = 0;
            followSpeed_R = 15;
        }
        //White Detected - Turn Right
        else {
            followSpeed_L = 15;
            followSpeed_R = 0;
        }
    }
}

// search task
task findLineTask(){
	while(true){
		if(lightValue_Avg > threshold){
			findSpeed_L = findSpeed_L  + 0.0007;
			findSpeed_R = findSpeed_R + 0.0003;

		}
		else{
      findSpeed_L = 0;
			findSpeed_R = 0;
			findLine = false;
		}
	}
}

task main()
{
	getLightValues();

	startTask(arbiterTask);
  startTask(findLineTask);
  startTask(followLineTask);
  startTask(avoidTask);

	while(true){
		getLightValues();
	}
}

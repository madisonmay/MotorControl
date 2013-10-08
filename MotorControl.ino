#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);
int sensorPin = A0;
long sensorValue = 0;
float range = 250.0;
float base = -range/2.0;
boolean reverse = false;
float current = 0;
float prevTime;
float prevPosition;

//control values
float p = 4;
float d = -20000;

//margin of error (in degrees)
float moe = 1;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(150);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);
  moveTo(0.0);
}

float getPosition(){
	sensorValue = analogRead(sensorPin);
	float position = sensorValue * range / 1023.0 + base;
	return position;	
}

float P_Control(float currentPos, float goalPosition){ //returns the necessary charge value assuming P Control
	float proportion = 255.0 / range; 	//Largest difference is 270 degrees
//and largest motor speed is 255.
	return (goalPosition - currentPos) * proportion;
}

float D_Control(float pos, float time){
        float vel = (pos - prevPosition)/(time - prevTime);
        
        //reset values
        prevTime = time;
        prevPosition = pos;
        
        return vel*d;
}
        

void moveTo(float goalPosition){
        prevTime = micros();
        prevPosition = getPosition();
        
	while(goalPosition - getPosition() > moe|| getPosition() - goalPosition > moe){
  
                float pos = getPosition();
                float time = micros();
                
                Serial.print(micros());
                Serial.print(",");
                Serial.print(pos);
                Serial.print("\n");
        
		float pControlValue = P_Control(pos, goalPosition);
                float dControlValue = D_Control(pos, time);
                float controlValue = pControlValue + dControlValue;

		float baseValue = 40; //Minimum addition to account for resistance
		if(controlValue < 0){
			myMotor->run(BACKWARD);
                        float velocity = -p * controlValue + baseValue;
			myMotor->setSpeed((velocity <= 255) ? velocity : 255);
		}
		else{
			myMotor->run(FORWARD);
                        float velocity = p * controlValue + baseValue;
			myMotor->setSpeed((velocity <= 255) ? velocity : 255);
		}
                 delay(5);
	}
	myMotor->setSpeed(0);
        float pos = getPosition();
        current = goalPosition;
      }
      
void iterate(float goalPosition){
        prevTime = micros();
        prevPosition = getPosition();
        
        float pos = getPosition();
        float time = micros();
                
        float pControlValue = P_Control(pos, goalPosition);
        float controlValue = pControlValue;

        float baseValue = 40; //Minimum addition to account for resistance
        if(controlValue < 0){
	    myMotor->run(BACKWARD);
            float velocity = -p * controlValue + baseValue;
	    myMotor->setSpeed((velocity <= 255) ? velocity : 255);
	} else {
	    myMotor->run(FORWARD);
            float velocity = p * controlValue + baseValue;
	    myMotor->setSpeed((velocity <= 255) ? velocity : 255);
        }
        current = getPosition();
        Serial.print(micros());
        Serial.print(",");
        Serial.print(current);
        Serial.print("\n");
      }

void rotate(float degree) {
  float minimum = base;
  float maximum = range + base;
  float pos = current;
  float new_pos = pos +  degree;
  
  //deal with extrema
  new_pos = (new_pos < maximum) ? new_pos : maximum;
  new_pos = (new_pos > minimum) ? new_pos : minimum;
  
  if (new_pos == minimum || new_pos == maximum) {
    Serial.println("Maximum range reached");
  }
  
  //actuate motor
  moveTo(new_pos);
}

void readSensor() {
  sensorValue = analogRead(sensorPin);
  Serial.print("Sensor Value: ");
  Serial.print(sensorValue);
  Serial.println();
  float position = sensorValue * range / 1023.0 + base;
  Serial.print("Position: ");
  Serial.print(position);
  Serial.println();
}

void sinMotion() {
    float t = millis()/1000.0;
    iterate(sin(t)*base);
}
  
void loop() {
  moveTo(90.0);
  delay(100);
  moveTo(-90.0);
  delay(100);
}



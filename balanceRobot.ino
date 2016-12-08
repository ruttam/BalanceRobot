#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_Sensor.h>
#include <stdio.h>

const int leftWheelPin1 = 7;
const int leftWheelPin2 = 6;
const int leftWheelVelocityPin3 = 5;
const int rightWheelPin1 = 8;
const int rightWheelPin2 = 9;
const int rightWheelVelocityPin3 = 10;

int leftWheel[3] = {leftWheelPin1, leftWheelPin2, leftWheelVelocityPin3};
int rightWheel[3] = {rightWheelPin1, rightWheelPin2, rightWheelVelocityPin3};

const int FORWARD = 1;
const int REVERSE = 0;

//Instantiate accelerometer/gyro
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

int readCount = 0;
int readingFrequency;
int bluetoothInput[3] = {0, 0, 0};
float x = 0;
char floatstr[15];
int printCount;

void setup(void) {
  Serial.begin(9600);
  pinMode(leftWheelPin1, OUTPUT);
  pinMode(leftWheelPin2, OUTPUT);
  pinMode(leftWheelVelocityPin3, OUTPUT);
  pinMode(rightWheelPin1, OUTPUT);
  pinMode(rightWheelPin2, OUTPUT);
  pinMode(rightWheelVelocityPin3, OUTPUT);
  if(!accel.begin()) {
    while(1);
  }
}

void loop(void) {
  float angle = readAccelerometerData();
  printCount++;
  if(printCount > 200) {
    Serial.println(angle);
    printCount = 0; 
  }

  //Reading input from serial
  if (Serial.available() > 0) {
    bluetoothInput[readCount] = Serial.read();

    if (readCount == 2)  {
      Serial.println("Conneted via Bluetooth: " + String(bluetoothInput[0]));
      readCount = 0;
      } else {
        readCount++;
      } 
    }

    //If input from serial "Go straight"
    if (bluetoothInput[1] == 97) {
    //TODO: Implementation
  }

  if (angle > 5) {
    moveWheel(leftWheel, FORWARD, 75);
    moveWheel(rightWheel, REVERSE, 75);
  } else if (angle < -5) {
    moveWheel(leftWheel, REVERSE, 75);
    moveWheel(rightWheel, FORWARD, 75);
  } else {
    moveWheel(leftWheel, FORWARD, 0);
    moveWheel(rightWheel, REVERSE, 0);
  }
}

float readAccelerometerData() {
  sensors_event_t accelEvent;
  accel.getEvent(&accelEvent);
  //Read every 200th data from accelerometer sensor
  if(readingFrequency == 20) {
    x = accelEvent.acceleration.x;
    readingFrequency = 0;
  } else {
    readingFrequency++;
  }

  return normalizeAngle(x);
}

void moveWheel(int wheel[3], int direction, int throttle) {
  digitalWrite(wheel[0], direction == FORWARD ? LOW : HIGH);
  digitalWrite(wheel[1], direction == FORWARD ? HIGH : LOW);
  digitalWrite(wheel[2], throttle);
}

float normalizeAngle(float angle) {
  return angle * 9;
}
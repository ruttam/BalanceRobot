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

         //BACWARD = false; //0
const bool FORWARD = true;  //1
const int LEFT = 2;
const int RIGHT = 3;
const int STOP = 4;

//Instantiate accelerometer/gyro
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

int readCount = 0;
int readingFrequency;
int bluetoothInput[3] = {0, 0, 0};
int bluetoothDirection;
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
  if (!accel.begin()) {
    while (1);
  }
}

void loop(void) {
  float angle = readAccelerometerData();
  //Serial.println(angle);
  bool isForward = setBalancingDirection(angle);
  moveWheel(leftWheel, isForward, calculateVelocity(angle));
  moveWheel(rightWheel, !isForward, calculateVelocity(angle));
  //Serial.print("Velocity: "); 
  //Serial.println(calculateVelocity(angle));
  //Reading input from serial
  if (Serial.available() > 0) {
    
  Serial.println(Serial.read());
    bluetoothDirection = setBluetoohDirection(readBluetoothData());
    int cycles = 1500;
    while(cycles > 0){
      Serial.print("Direction");
      Serial.println(bluetoothDirection);
    if(bluetoothDirection == 0 || bluetoothDirection == 1) {
      moveWheel(leftWheel, bluetoothDirection, 225);
      moveWheel(rightWheel, !bluetoothDirection, 225);
    } else {
      if(bluetoothDirection == 2){
        rotateWheel(leftWheel, true);
        rotateWheel(rightWheel, false);
      } else {
        rotateWheel(leftWheel, false);
        rotateWheel(rightWheel, true);
      }
    }
    cycles--;
    }
  }
}

float readAccelerometerData() {
  sensors_event_t accelEvent;
  accel.getEvent(&accelEvent);
  x = accelEvent.acceleration.x;
  return normalizeAngle(x);
}

int readBluetoothData() {
  bluetoothInput[readCount] = Serial.read();
  if (readCount == 2)  {
    Serial.println("Conneted via Bluetooth: " + String(bluetoothInput[0]));
    return bluetoothInput[0];
    readCount = 0;
  } else {
    readCount++;
  }
}

void rotateWheel(int wheel[3], bool move) {
  digitalWrite(wheel[0], move == true ? LOW : LOW);
  digitalWrite(wheel[1], move == true ? HIGH : LOW);
  analogWrite(wheel[2], move == true ? 180 : 0);
}

void moveWheel(int wheel[3], bool direction, int throttle) {
  digitalWrite(wheel[0], direction == FORWARD ? LOW : HIGH);
  digitalWrite(wheel[1], direction == FORWARD ? HIGH : LOW);
  analogWrite(wheel[2], throttle);
}

float normalizeAngle(float angle) {
  return angle * 9;
}

bool setBalancingDirection(float angle) {
  return angle > 0 ? true : false;
}

int calculateVelocity(float angle){
  if(angle != 0 && angle < 45 && angle > -45) {
    int result = angle > 0 ? angle * 5 : (-1) * (angle * 5);
    return result > 255 ? 255 : result;
  } else {
    return 0;
  }
}

int setBluetoohDirection(int input){
  //If input from serial "Go forward", else if "Go left", else if "Stop", else if "Go right", else if "Go backward"
  return bluetoothInput[1] == 97 ? 1 : bluetoothInput[1] == 98 ? LEFT : bluetoothInput[1] == 99 ? STOP : bluetoothInput[1] == 100 ? RIGHT : 0;
}

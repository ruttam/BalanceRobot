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

const bool FORWARD = true;
const int LEFT = 3;
const int RIGHT = 4;

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
  if (!accel.begin()) {
    while (1);
  }
}

void loop(void) {
  float angle = readAccelerometerData();
  Serial.println(angle);
  bool isForward = setBalancingDirection(angle);
  moveWheel(leftWheel, isForward, calculateVelocity(angle));
  moveWheel(rightWheel, !isForward, calculateVelocity(angle));
  //Reading input from serial
  if (Serial.available() > 0) {
    setBluetoohDirection(readBluetoothData());
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
  if(angle != 0 && (angle < 45 || angle > -45)) {
    return angle > 0 ? angle * 5 : (-1) * (angle * 5);
  } else {
    return 0;
  }
}

int setBluetoohDirection(int input){
  //If input from serial "Go straight"
  if (bluetoothInput[1] == 97) {
    //TODO: Implementation
  }
}

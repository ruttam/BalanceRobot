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

const int FORWARD = 1;
const int REVERSE = 0;

//Instantiate accelerometer/gyro
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

int readCount = 0;
int count;
int bluetoothInput[3] = {0, 0, 0};
float x = 0;
char floatstr[15];

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
  readAccelerometerData();

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

    //If falling to the back - go to the front
  if (x < -1 && x > -4) {
      //moveWheel(
    digitalWrite(leftWheelPin1, HIGH);
    digitalWrite(leftWheelPin2, LOW);
    digitalWrite(leftWheelVelocityPin3, 20);
    digitalWrite(rightWheelPin1, LOW);
    digitalWrite(rightWheelPin2, HIGH);
    digitalWrite(rightWheelVelocityPin3, 20);
    //If fell to the back - do not move
  } else if(x < -5) {
    digitalWrite(leftWheelPin1, LOW);
    digitalWrite(leftWheelPin2, LOW);
    digitalWrite(leftWheelVelocityPin3, 0);
    digitalWrite(rightWheelPin1, LOW);
    digitalWrite(rightWheelPin2, LOW);
    digitalWrite(rightWheelVelocityPin3, 0);
    //If falling to the front - go back
  } else if (x > 1 && x < 4) {
    digitalWrite(leftWheelPin1, LOW);
    digitalWrite(leftWheelPin2, HIGH);
    digitalWrite(leftWheelVelocityPin3, 20);
    digitalWrite(rightWheelPin1, HIGH);
    digitalWrite(rightWheelPin2, LOW);
    digitalWrite(rightWheelVelocityPin3, 20);
    //If fell to the front - do not move
  } else if(x > 5) {
    digitalWrite(leftWheelPin1, LOW);
    digitalWrite(leftWheelPin2, LOW);
    digitalWrite(leftWheelVelocityPin3, 0);
    digitalWrite(rightWheelPin1, LOW);
    digitalWrite(rightWheelPin2, LOW);
    digitalWrite(rightWheelVelocityPin3, 0);
    //If standing straight - do not move
  } else if(x > -2 && x <2){
    digitalWrite(leftWheelPin1, LOW);
    digitalWrite(leftWheelPin2, LOW);
    digitalWrite(leftWheelVelocityPin3, 0);
    digitalWrite(rightWheelPin1, LOW);
    digitalWrite(rightWheelPin2, LOW);
    digitalWrite(rightWheelVelocityPin3, 0);
  }
}

void readAccelerometerData(void) {
  sensors_event_t accelEvent;
  accel.getEvent(&accelEvent);
    //Read every 200th data from accelerometer sensor
  if(count == 200) {
    x = accelEvent.acceleration.x;
    printFloat("Accelerometer x value:", normalizeAngle(x));
    count = 0;
  } else {
    count++;
  }
}

void moveWheel(int wheel, int direction, int throttle) {
  if(throttle > 0){
    digitalWrite(wheel, direction == FORWARD ? LOW : HIGH);
    digitalWrite(wheel, direction == FORWARD ? HIGH : LOW);
    digitalWrite(wheel, throttle);
  } else {
    digitalWrite(wheel, LOW);
    digitalWrite(wheel, LOW);
    digitalWrite(wheel, 0);
  }
}

float normalizeAngle(float angle) {
  //We multiply reading by 9 to get normalized value.
  return angle * 9;
}

void printFloat(char *label, float value) {
    //Since printf() does not include float or double numbers printing
    //we need to convert float to string with dtostrf() function
    //http://www.hobbytronics.co.uk/arduino-float-vars
  dtostrf(x, 7, 3, floatstr);
    //TODO: check if we need to extract value from pointer or not. (On the spot)
  Serial.printf("%s %s \n", &label, floatstr);
}
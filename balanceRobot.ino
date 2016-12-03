#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_Sensor.h>
#include <stdio.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
int leftWheelPin1 = 7;
int leftWheelPin2 = 6;
int leftWheelVelocityPin3 = 5;
int rightWheelPin1 = 8;
int rightWheelPin2 = 9;
int rightWheelVelocityPin3 = 10;
int i = 0;
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
  if(!accel.begin())
  {
    while(1);
  }
}

void loop(void) {
  readAccelerometerData();
  //Reading input from telephone
  if(Serial.available() > 0){
    bluetoothInput[i] = Serial.read();
    if(i == 2){
      Serial.println("Conneted via Bluetooth: " + String(bluetoothInput[0]));
      i = 0;
    } else {
        i++;
    }
  }
  //If input from telephone "Go straight"
  if(bluetoothInput[1] == 97){
    //TODO: Implementation
  }
  //If falling to the back - go to the front
  if(x < -1 && x > -4) {
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
  } else if(x > 1 && x < 4) {
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
    //Since printf() does not include float or double numbers printing
    //we need to convert float to string with dtostrf() function
    //http://www.hobbytronics.co.uk/arduino-float-vars
    dtostrf(x, 7, 3, floatstr);
    //In order to use printf() we need to implement it in Print.h header file in Arduino cores
    //http://playground.arduino.cc/Main/Printf
    Serial.printf("Accelerometer x value: %s \n", floatstr);
    count = 0;
  } else {
    count++;
  }
}

//TODO: Implementation
void moveWheel(int wheel, int input, int velocity, int velocityLevel){
  if(input == HIGH){
    digitalWrite(wheel, LOW);
    digitalWrite(wheel, HIGH);
    digitalWrite(wheel, velocityLevel);
  } else {
    digitalWrite(wheel, LOW);
    digitalWrite(wheel, LOW);
    digitalWrite(wheel, 0);
  }
}

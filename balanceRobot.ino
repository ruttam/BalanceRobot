#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_Sensor.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
int leftWheelPin1 = 7;
int leftWheelPin2 = 6;
int leftWheelVelocityPin3 = 5;
int rightWheelPin1 = 8;
int rightWheelPin2 = 9;
int rightWheelVelocityPin3 = 10;
int i = 0;
int count;
int state[3] = {0, 0, 0};

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
  sensors_event_t accelEvent;
  accel.getEvent(&accelEvent);
    //Read every 10th data from 3D axis sensor
  if(count == 200){
    Serial.print("Accelerometer: ");
    Serial.print(accelEvent.acceleration.x);
    Serial.print("   ");
    Serial.print(accelEvent.acceleration.y);
    Serial.print("   ");
    Serial.print(accelEvent.acceleration.z);
    Serial.println();
    count = 0;
  } else {
    count++;
  }
      //Reading input from telephone
    if(Serial.available() > 0){
      state[i] = Serial.read();
      if(i == 2){
        Serial.println("Conneted via Bluetooth: " + String(state[0]));
        i = 0;
      } else {
        i++;
      }
    }
    //If input from telephone "Go straight"
    if(state[1] == 97)
    if(accelEvent.acceleration.x < -1.1 && accelEvent.acceleration.x > -5) {
      digitalWrite(leftWheelPin1, LOW);
      digitalWrite(leftWheelPin2, HIGH);
      digitalWrite(rightWheelPin1, HIGH);
      digitalWrite(rightWheelPin2, LOW);
      digitalWrite(rightWheelVelocityPin3, 120);
    } else if(accelEvent.acceleration.x < -5) {
      digitalWrite(leftWheelPin1, LOW);
      digitalWrite(leftWheelPin2, LOW);
      digitalWrite(rightWheelPin1, HIGH);
      digitalWrite(rightWheelPin2, LOW);
      digitalWrite(rightWheelVelocityPin3, 255);
    } else if(accelEvent.acceleration.x > 1.1 && accelEvent.acceleration.x < 5) {
      digitalWrite(leftWheelPin1, HIGH);
      digitalWrite(leftWheelPin2, LOW);
      digitalWrite(rightWheelPin1, LOW);
      digitalWrite(rightWheelPin2, HIGH);
      digitalWrite(rightWheelVelocityPin3, 255);
    } else if(accelEvent.acceleration.x > 5) {
      digitalWrite(leftWheelPin1, LOW);
      digitalWrite(leftWheelPin2, HIGH);
      digitalWrite(rightWheelPin1, LOW);
      digitalWrite(rightWheelPin2, LOW);
      digitalWrite(rightWheelVelocityPin3, 0);
    } else {
      digitalWrite(leftWheelPin1, HIGH);
      digitalWrite(leftWheelPin2, LOW);
      digitalWrite(rightWheelPin1, HIGH);
      digitalWrite(rightWheelPin2, LOW);
      digitalWrite(rightWheelVelocityPin3, 255);
    }
    count = 0;
  } 
}

void read3D

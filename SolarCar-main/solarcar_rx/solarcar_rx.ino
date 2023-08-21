 // Arduino nano receiver code to control RC car using nRF24L01+PA+LNA transceiver
// and TB6612FNG motor driver
// Made for K-State Solar Club

#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include "printf.h"
#include <Wire.h>
#include <AccelStepper.h>
#include <Servo.h>

Servo myServo;
int servoPos = 0;

unsigned long trackingTimer;
AccelStepper stepMotor(4, 2, 4, 3, 7);
int stepPos;
float stepSpeed = 250.0;
unsigned long stepTimeout;

RF24 radio(9, 8); // CE, CSN
const byte address1[10] = "CAR_TX";
const byte address2[10] = "TELEMETRY";

// These pins are used to control motor direction
#define AIN1 A0
#define AIN2 A1
#define BIN1 A2
#define BIN2 A3
#define leftMotorPin 5
#define rightMotorPin 6

unsigned long timeout;  // Timer for timeout caused by lack of data from radios

float lightVoltages[4];
float batVoltage;
float batScale = 22.0 / (22.0+22.0);
float solarVoltage;
float solarScale = 22.0 / (22.0+47.0);
float rail_5v;
float inCurr;

unsigned long telemetryTimer;
struct radioData {
  float lightVals[4];
  float bat;
  float solar;
  float rail5V;
  float inCurr;
};
radioData telemetryData;

#define ADC_ADDR 0x33

// Code to configure/setup MAX11614 ADC
void startADC() {
  Wire.beginTransmission(ADC_ADDR);
  // Tells ADC to use internal CLK, external reference, and unipolar conversion
  // Also resets config register
  Wire.write(0xA2);
  Wire.endTransmission(); 
}

// Read single-ended channel of ADC
unsigned int readADC(int channel) {
  unsigned int val = 0x0000;
  unsigned int buf = 0x0000;
  char config = 0x61 | (channel<<1); 

  // Tell ADC which channel to start conversion on
  Wire.beginTransmission(ADC_ADDR);
  Wire.write(config);
  Wire.endTransmission();

  // First 4 bytes of 1st response are 1111, second 4 are 4 MSBs of data, all of second byte is LSB
  // Therefore data = response1[4:7] + response2
  Wire.requestFrom(ADC_ADDR, 2);  // Request two bytes from ADC
  while(2 > Wire.available()) {}  // Wait for two bytes to arrive
  buf = Wire.read();
  val = buf<<8;                 // Load left 8 bits with first buf response
  buf = Wire.read();
  val = (val & 0x0FFF) | buf;   // Load right 8 bits with second buf response
  return val;
}

// Convert 12-bit ADC output into voltage
float computeVoltsADC(unsigned int val, float ref) {
  float v;
  v = float(val) / 4096.0 * ref;
  return v;
}

void setup() {

  Wire.begin(); // Start I2C controller on Arduino
  startADC();   // Initialize ADC

  myServo.write(0);
  myServo.attach(10);

  stepMotor.setMaxSpeed(600);
  stepMotor.setAcceleration(1000.0);
  
  stepMotor.setSpeed(600);
  //stepMotor.moveTo(2038);

  // motor related pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);
  pinMode(rightMotorPin, OUTPUT);
  
  Serial.begin(9600);

  // Set up nRF24L01
  radio.begin();
  radio.setAutoAck(false);
  radio.openWritingPipe(address2);
  radio.openReadingPipe(1, address1);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  printf_begin();
  radio.printDetails();

  trackingTimer = millis();
  timeout = millis();
  telemetryTimer = millis();
  stepTimeout = millis();
}

void loop() {
  /*
  // I2C testing:
  Serial.println("ADC vals:");
  for (int i = 0; i<8; i++) {
    Serial.println(computeVoltsADC(readADC(i), 5),3);
  }
  Serial.println();
  delay(500);
  */
  // ADC function use:
  // float volts = computeVoltsADC(readADC(int channel), float v_ref);
  
  
  if (millis() - telemetryTimer >= 100) {
    
    for (int i = 0; i<4; i++) {
      lightVoltages[i] = computeVoltsADC(readADC(i),5);
    }
    
    rail_5v = computeVoltsADC(readADC(7),5);
    batVoltage = computeVoltsADC(readADC(5),5) / batScale;
    solarVoltage = computeVoltsADC(readADC(4),5) / solarScale;
    inCurr = (computeVoltsADC(readADC(6),5) - 2.5) / 0.4; // Current in amps


    
    memcpy(telemetryData.lightVals, lightVoltages, sizeof(lightVoltages));
    telemetryData.bat = batVoltage;
    telemetryData.solar = solarVoltage;
    telemetryData.rail5V = rail_5v;
    telemetryData.inCurr = inCurr;
    radio.stopListening();
    radio.write(&telemetryData, sizeof(telemetryData));
    radio.startListening();
    
    telemetryTimer += 100;
  }

  if (radio.available()) {
    //Serial.println("Data received");
    int txt[4] = {};
    radio.read(&txt, sizeof(txt));
    //Serial.println(txt[2]);

    // Normal mode
    if (txt[2] == 1) {

      //Serial.println("AUTOMATIC TRACKING MODE");
      
      // Check for direction of left motor
      if (txt[0] < 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
      }
      else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
      }
      // Check for direction of right motor
      if (txt[1] < 0) {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
      }
      else {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
      }
  
      // Write motor vals to PWM pins
      analogWrite(leftMotorPin, map(abs(txt[0]), 0, 512, 0, 255));
      analogWrite(rightMotorPin, map(abs(txt[1]), 0, 512, 0, 255));

      if (millis() - trackingTimer >= 50) {
        // lightVoltages[i]
    //    float maxLightVoltage = 0;
    //    int i;
    //    for (i = 0; i < 4; i++) {
    //      if (lightVoltages[i] >= maxLightVoltage) {
    //        maxLightVoltage = lightVoltages[i];
    //      }
    //    }
    
        // LDR locations
        // 0 | 1
        // -----
        // 2 | 3
        
        float avgL, avgR, avgU, avgD; // Average left, right, up, down
        avgL = ( lightVoltages[0] + lightVoltages[2] ) / 2.0;
        avgR = ( lightVoltages[1] + lightVoltages[3] ) / 2.0;
        avgU = ( lightVoltages[0] + lightVoltages[1] ) / 2.0;
        avgD = ( lightVoltages[2] + lightVoltages[3] ) / 2.0;
    
    
        int servoDir;   // +1 is up, -1 is down
        int stepDir;    // +1 is right, -1 is 
        float tol = 0.1; // Tolerance in volts
    
        if (abs(avgL - avgR) < tol) {
          stepDir = 0;
        }
        else {
          if (avgL > avgR) {
            stepDir = -1;
          }
          else {
            stepDir = 1;
          }
        }
    
        if (abs(avgU-avgD) < tol) {
          servoDir = 0;
        }
        else {
          if (avgU < avgD) {
            servoDir = 1;
          }
          else {
            servoDir = -1;
          }
        }
    
    
        
        // servoPos from 0 degrees to 180 degrees
        // stepPos from 0 to 2038
    
        servoPos += servoDir*2;
      
    
        // Check servo edge cases
        if (servoPos < 0) {
          servoPos = 0;
        } 
        else {
          if (servoPos > 180) {
            servoPos = 180;
          }
          else {
            myServo.write(servoPos);
          }
        }
    
        //stepMotor.move(dir*5);
        if (stepMotor.distanceToGo() == 0) {
          stepMotor.move(stepDir*10);
          //stepMotor.setSpeed(stepDir*stepSpeed);
        }

        trackingTimer += 50;
      }
      
    } 

    // Manual solar tracker mode
    else {

      //Serial.println("MANUAL TRACKING MODE");
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, LOW);

      analogWrite(leftMotorPin, 255);
      analogWrite(rightMotorPin, 255);

      //Serial.print(txt[0]); Serial.print(", "); Serial.println(txt[1]);

      if (abs(txt[0]) > 50) {
        if (txt[0] < 0) {
          if (stepMotor.distanceToGo() == 0) {
            stepMotor.move(50);
            //stepMotor.setSpeed(stepSpeed);
          }
        }
        else {
          if (stepMotor.distanceToGo() == 0) {
            stepMotor.move(-50);
            //stepMotor.setSpeed(-stepSpeed);
          }
        }
      }
      //stepMotor.runSpeedToPosition();

      if (abs(txt[1]) > 50) {
        if (txt[1] < 0) {
          servoPos += 1;
        }
        else {
          servoPos -= 1;
        }  
      }

      
      // Check servo edge cases
      if (servoPos < 0) {
        servoPos = 0;
      } 
      else {
        if (servoPos > 180) {
          servoPos = 180;
        }
        else {
          myServo.write(servoPos);
        }
      }
      
      
    }


    
    timeout = millis();
  } // end of if radio.available()
  
  else {
    if ( (millis() - timeout >= 500)) {  // If it's been 500ms since last radio transmission
      // should put motor driver into "stop" mode
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, LOW);
      analogWrite(leftMotorPin, 255);
      analogWrite(rightMotorPin, 255);
    }
  } // end of timeout 

  
 
          //stepMotor.runSpeedToPosition();
          stepMotor.run();
          if (stepMotor.distanceToGo() == 0 && millis() - stepTimeout > 50) {
            stepMotor.disableOutputs();
            stepTimeout = millis();
          }
        
    


  
  
} // End of main loop

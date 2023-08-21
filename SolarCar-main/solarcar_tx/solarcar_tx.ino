// Arduino nano transmitter code to control RC car using nRF24L01+PA+LNA transceiver
// and TB6612FNG motor driver
// Made for K-State Solar Club

#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include "printf.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Included in the GitHub TX Folder
#include "ButtonDebounce.h"
#include "EncoderMonitor.h"

#define LEFT A1
#define RIGHT A0

// LCD I2C address: 0x3C
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


struct radioData {
  float lightVals[4];
  float bat;
  float solar;
  float rail5V;
  float inCurr;
};

RF24 radio(9, 8); // CE, CSN
const byte address1[10] = "CAR_TX";
const byte address2[10] = "TELEMETRY";

int data[4];
unsigned long timer;
int interval = 20; // interval between data transmissions
unsigned long ledTimer;

/*
 Pin Asssingemnts:
 
 Button in center of Board: D5
 
 Middle Switch (SW1) D6
 
 Knob A7
 
 Left Joystick A3
 
 Right Joystick A2
 
*/ 

void setup() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Screen failed to begin");
  }
  
  //Initialize Interface 
  EncoderInitialize();
  ButtonInitialize(); 
  
  // LCD testing
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10,25);
  display.println("KSU SOLAR CLUB 2023");
  display.display();
  delay(500);
  Serial.begin(9600);
  pinMode(4, OUTPUT);
  pinMode(6, INPUT);

  // Set up nRF24L01
  radio.begin();
  radio.setAutoAck(false);
  radio.openWritingPipe(address1);
  radio.openReadingPipe(1, address2);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_MIN);
  //radio.stopListening();
  printf_begin();
  //radio.printDetails();
  Serial.println("Starting");

  timer = millis();
}

void loop() {

  if (radio.available()) {
    radioData telemetryData;
    radio.read(&telemetryData, sizeof(telemetryData));
    Serial.println(telemetryData.bat);

    display.clearDisplay();
    display.setCursor(0,0);
    display.print("Bat:   " + String(telemetryData.bat, 3)+ "V");
    display.setCursor(0,10);
    display.print("Solar: " + String(telemetryData.solar, 3) + "V");
    display.setCursor(0,20);
    display.print("Buck:  " + String(telemetryData.rail5V, 3) + "V");
    display.setCursor(0,30);
    display.print("I_in:  " + String(telemetryData.inCurr, 3) + "A");
    display.setCursor(0,40);
    display.print("L0: " + String(telemetryData.lightVals[0],2) + 
                 " L1: " + String(telemetryData.lightVals[1],2));
    display.setCursor(0,50);
    display.print("L2: " + String(telemetryData.lightVals[2],2) + 
                 " L3: " + String(telemetryData.lightVals[3],2));

    display.setCursor(100, 0);
    if (digitalRead(6)) { 
      display.print("AUTO");      
    }
    else {
      display.print("MAN");
    }
    display.display();
  }
  
  if (millis() - timer >= interval) {
    data[0] = analogRead(LEFT) - 512; // Left motor
    data[1] = analogRead(RIGHT) - 512; // Right motor
    if (digitalRead(6)) {
      data[2] = 1;
    }
    else {
      data[2] = 0;
    }
    data[3] = 0;  // same as above
    radio.stopListening();
    radio.write(&data, sizeof(data));
    radio.startListening();
    timer += interval;
    //Serial.print("Left: "); Serial.print(data[0]); Serial.print(", Right: "); Serial.println(data[1]);
  }

  if (millis() - ledTimer >= 1000) {
    PORTD ^= 0x10;
    ledTimer += 1000;
  }
  
}

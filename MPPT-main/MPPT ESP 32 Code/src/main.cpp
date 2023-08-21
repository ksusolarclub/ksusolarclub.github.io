#include <Arduino.h>
#include "Adafruit_ADS1X15.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);
unsigned long lcdTimer;

#define SS 1
#define MOSI 7
#define MISO 2
#define SCK 0

#define SDA 5
#define SCL 6

#define SHUTDOWN 4
#define REVERSE 10
#define PWM_PIN 3

#define MAX_CURRENT 2.0 // Maximum current that MPPT can output
#define TARGET_VOLTAGE 12.6 // Target voltage for LiPo battery
#define STEP_SIZE 0.01 // Step size for current control
#define USER_CURRENT 1.0 // User-defined charging current
#define MAX_TRIES 5 // Maximum tries to fix overcurrent
#define VOLTAGE_TOLERANCE 0.1 // Tolerance for voltage readings


enum stateTypes {
  INITIAL,
  MPPT,
  OVERCURRENT,
  STOP
};
stateTypes chargeState = INITIAL;

int overCurrentTries;
int reverseFlag = 0;
unsigned long currentTime;
unsigned long SD_timer;

// two ADC addresses 0x48 and 0x49
TwoWire CustomI2C = TwoWire(0);
Adafruit_ADS1015 ads0;
Adafruit_ADS1015 ads1;
float adc0_voltage[4], adc1_voltage[4];
int16_t adc0_val[4], adc1_val[4];



float inputVoltage, outputPowerOld, inputCurrent, batVoltage, batCurrent, loadCurrent, refVoltage, inputPower, outputPower;
const int filterLen = 8;
float newCurr;
float inCurrFilter[filterLen], batCurrFilter[filterLen], loadCurrFilter[filterLen] = {0};
int filtIndex = 0;
float inputScale = 2.2 / (2.2+15);  // Voltage divider for input voltage
float outputScale = 2.2 / (2.2+15); // Voltage divider for battery voltage
float eff;  // Efficiency
float effOld;

int dutyCycle = 75;
int DC_direction = 1;

// https://gist.github.com/benpeoples/3aa57bffc0f26ede6623ca520f26628c states frequency vs resolution
const int freq = 75000;   // Seemed to be working with 20 kHz also
const int pwmChannel = 0;
const int resolution = 8;

SPIClass SPI_SD(FSPI);
String filename;
int SD_flag = 1;



void MPPT_algorithm() {
  if (outputPowerOld > outputPower) {
    DC_direction *= -1;
  }

 
  else {
    if (dutyCycle <= 0) {
    dutyCycle = 1;
    } 
    else if (dutyCycle >=250) {
      dutyCycle = 249;
    }
  }
}

void setPWM() {
    if (dutyCycle > 0 && dutyCycle < 250) {
    dutyCycle += DC_direction;

    outputPowerOld = outputPower;
    ledcWrite(pwmChannel, dutyCycle);
  }
}


void setup() {
  // put your setup code here, to run once:
  //gpio_reset_pin(GPIO_NUM_3(3));
  
  //pinMode(PWM_PIN, OUTPUT);
  pinMode(SHUTDOWN, OUTPUT);
  pinMode(REVERSE, OUTPUT);

  digitalWrite(SHUTDOWN, LOW);
  digitalWrite(REVERSE, LOW);
  //digitalWrite(PWM_PIN, LOW);

  Serial.begin(115200);
  delay(1000);



  // Set up both ADCs
  CustomI2C.begin(SDA, SCL, 100000);
  if (!ads0.begin(0x48, &CustomI2C)) { // Address according to ESP32
    Serial.println("Failed to initialize ADC 0x48");
    while (1);
  }
  if (!ads1.begin(0x49, &CustomI2C)) { // Address according to ESP32
    Serial.println("Failed to initialize ADC 0x49");
    while (1);
  }
  ads0.setDataRate(RATE_ADS1015_3300SPS);
  ads0.setGain(GAIN_TWOTHIRDS);
  ads1.setDataRate(RATE_ADS1015_3300SPS);
  ads1.setGain(GAIN_TWOTHIRDS);
  Serial.println("Both ADCs initiated!");

  // Initialize LCD
  lcdTimer = millis();
  lcd.init(&CustomI2C);
  Serial.println("Here");
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("KSU SOLAR CLUB REV1");
  lcd.setCursor(0,2);
  lcd.print(" FIRMWARE VER: 1.0 ");


  // SD card test
  SPI_SD.begin(SCK, MISO, MOSI, SS);
  delay(100);
  filename = "/0.csv";
  if (!SD.begin(SS, SPI_SD)) {
    Serial.println("Failed to initialize SD card");
    SD_flag = 0;
    delay(500);
    //while (1);
  }
  int filenum = 0;
  while (SD.exists(filename)) {
    filename = "/" + String(filenum) + ".csv";
    filenum++;
  }
  //File dir =  SD.open(filename, FILE_APPEND);  // Don't use FILE_WRITE unless you want to lose data
  //dir.println("testing 123 3/6/2023");     // also a print() function
  //dir.close();

  //dutyCycle = 0;
  outputPowerOld = -1;

  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(PWM_PIN, pwmChannel);
  ledcWrite(pwmChannel, dutyCycle);

  //digitalWrite(PWM_PIN, HIGH);
  //digitalWrite(SHUTDOWN, HIGH); //When low shutsdown system, when high its on
  //digitalWrite(REVERSE, HIGH); // if battery voltage > input voltage shutdown system. Turn reverse pin on
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // probably don't need this delay(10);
  for (int i=0; i<4; i++) {
    adc0_val[i] = ads0.readADC_SingleEnded(i);
    adc1_val[i] = ads1.readADC_SingleEnded(i);
  }

  // Convert all values to voltage
  refVoltage = ads1.computeVolts(adc1_val[1]);  // Should be 5V
  inputVoltage = ads0.computeVolts(adc0_val[0]) / inputScale;

  // Moving average filter for input current
  /*
  inputCurrent = 0;
  inCurrFilter[filtIndex] = ( ads0.computeVolts(adc0_val[1]) - (refVoltage/2) ) / 0.4; // Units in Amps
  for (int i = 0; i < filterLen; i++) {
    inputCurrent += inCurrFilter[i];
  }
  inputCurrent = inputCurrent / filterLen;
  */

  inputCurrent -= inCurrFilter[filtIndex] / float(filterLen);
  inCurrFilter[filtIndex] = ( ads0.computeVolts(adc0_val[1]) - (refVoltage/2.0) ) / 0.4; // Units in Amps
  inputCurrent += inCurrFilter[filtIndex] / float(filterLen);
  
  batVoltage = ads1.computeVolts(adc1_val[3]) / outputScale;

  // Moving average filter for battery current
  /*
  batCurrent = 0;
  batCurrFilter[filtIndex] = ( ads1.computeVolts(adc1_val[2]) - (refVoltage/2) ) / 0.4; // Units in Amps
  for (int i = 0; i < filterLen; i++) {
    batCurrent += batCurrFilter[i];
  }
  batCurrent = batCurrent / filterLen;
  */
 batCurrent -= batCurrFilter[filtIndex] / float(filterLen);
 batCurrFilter[filtIndex] = ( ads1.computeVolts(adc1_val[2]) - (refVoltage/2.0) ) / 0.4; // Units in Amps
 batCurrent += batCurrFilter[filtIndex] / float(filterLen);




  // Moving average filter for load current
  loadCurrent -= loadCurrFilter[filtIndex] / float(filterLen);
  loadCurrFilter[filtIndex] = ( ads1.computeVolts(adc1_val[0]) - (refVoltage/2.0) ) / 0.4; // Units in Amps
  loadCurrent += loadCurrFilter[filtIndex] / float(filterLen);

  /*
  loadCurrent = 0;
  loadCurrFilter[filtIndex] = ( ads1.computeVolts(adc1_val[0]) - (refVoltage/2) ) / 0.4;
  for (int i = 0; i < filterLen; i++) {
    loadCurrent += loadCurrFilter[i];
  }
  loadCurrent = loadCurrent / filterLen;
  */

  inputPower = inputVoltage * inputCurrent;
  outputPower = batVoltage * (batCurrent + loadCurrent);
  eff = outputPower / inputPower;

  currentTime = millis();

  // Change moving average filter index
  filtIndex++;
  if (filtIndex >= filterLen) {
    filtIndex = 0;
  }

  switch (chargeState) {
    case INITIAL:
      if (batVoltage > 0.5 && inputVoltage > batVoltage && batVoltage < TARGET_VOLTAGE) {
        chargeState = MPPT;
        reverseFlag = 0;
        digitalWrite(REVERSE, HIGH);
        digitalWrite(SHUTDOWN, HIGH);
      }
      break;

    // Run MPPT algorithm
    case MPPT:
      if (batVoltage > TARGET_VOLTAGE) {    // Check for overvoltage
        chargeState = STOP;
      }
      else if (batCurrent > MAX_CURRENT) {  // Check for overcurrent
        chargeState = OVERCURRENT;
        overCurrentTries = 0;
        DC_direction = -1;
      }
      else if (batVoltage > inputVoltage) { // Check for reverse current
        reverseFlag = 1;
        digitalWrite(REVERSE, LOW);
        digitalWrite(SHUTDOWN, LOW);
        chargeState = INITIAL;
      }
      else {
        MPPT_algorithm();
      }
      setPWM();
      break;

    // Attempt to fix overcurrent, if not fixed after MAX_TRIES, stop charging
    case OVERCURRENT:
      if (overCurrentTries > MAX_TRIES) {
        chargeState = STOP;
      }
      else if (batCurrent < MAX_CURRENT) {
        chargeState = MPPT;
      }
      else if (batVoltage > inputVoltage) { // Check for reverse current
        reverseFlag = 1;
        digitalWrite(REVERSE, LOW);
        digitalWrite(SHUTDOWN, LOW);
        chargeState = INITIAL;
      }
      setPWM();
      overCurrentTries++;
      break;

    // Stop charging
    case STOP:
      digitalWrite(REVERSE, LOW);
      digitalWrite(SHUTDOWN, LOW);

      // If battery needs charged again
      if (batVoltage < TARGET_VOLTAGE - VOLTAGE_TOLERANCE) {
        chargeState = INITIAL;
        reverseFlag = 0;
      }
      break;
  }

  if (millis() - lcdTimer >= 100) {
    //lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("In:  " + String(inputVoltage, 3) + "V " + String(abs(inputCurrent), 3) + "A  ");
    lcd.setCursor(0,1);
    lcd.print("Out: " + String(batVoltage, 3) + "V " + String(abs(batCurrent), 3) + "A  ");
    lcd.setCursor(0,2);
    lcd.print("DC%: " + String(float(dutyCycle)/255.0*100,1));
    lcd.setCursor(9,2);

    if (eff >= 1) {
      lcd.print(" Eff: Uhhh");
    }
    else {
      lcd.print(" Eff: " + String(eff*100.0,1).substring(0,4));
    }

    lcd.setCursor(0,3);
    switch (chargeState) {
      case INITIAL:
        if (reverseFlag = 1) {lcd.print("REVERSE");}
        else                 {lcd.print("INITIAL");}
        break;
      case MPPT:
        lcd.print("MPPT   ");
        break;
      case OVERCURRENT:
        lcd.print("CURRENT");
        break;
      case STOP:
        lcd.print("STOPPED");
        break;
    }

    lcd.print("   Ref: " + String(refVoltage,2));

    lcdTimer = millis();
  }

  // Write data to SD card every 500 ms
  if (millis() - SD_timer >= 500) {

      // Write to SD card
    if (SD_flag) {
      File dir =  SD.open(filename, FILE_APPEND);  // Don't use FILE_WRITE unless you want to lose data
      dir.println(String(inputVoltage, 3) + "," + String(inputCurrent, 3) + "," + String(batVoltage, 3) + "," + String(batCurrent, 3)
                + "," + String(dutyCycle) + "," + String(eff, 3) + "," + String(currentTime) + "," + String(chargeState));    // also a print() function
      dir.close();
    }
    SD_timer += 500;
  }

  /*
  Serial.print("5V rail: "); Serial.print(refVoltage,3);
  Serial.print(", Input volt: "); Serial.print(inputVoltage, 3);
  Serial.print(", Input curr: "); Serial.print(inputCurrent, 3);
  Serial.print(", Bat volt: "); Serial.print(batVoltage, 3);
  Serial.print(", Bat curr: "); Serial.print(batCurrent, 3);
  Serial.print(", Load curr: "); Serial.print(loadCurrent, 3);
  Serial.print(", DutyCycle: "); Serial.print(dutyCycle);
  Serial.print(", Eff: "); Serial.println(outputPower/inputPower*100,3);
  */

} // End of loop()

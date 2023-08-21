#include <Arduino.h>
// include the SD library:
#include <SPI.h>
#include <SD.h>
 
// WeMos D1 esp8266: D8 as standard
/*
This example shows how use the utility libraries
 
  The circuit:
    SD card attached to SPI bus as follows:
        SS    = 10;
        MOSI  = 7;
        MISO  = 2;
        SCK   = 6;
 
 
   by Mischianti Renzo <https://www.mischianti.org>
  
   https://www.mischianti.org


*/
// include the SD library:
const int chipSelect = SS;
uint8_t buf[20];

void printDirectory(File dir, int numTabs);
 
void setup() {

  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
 
 
  Serial.print("\nInitializing SD card...");
 
  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!SD.begin(SS)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    while (1);
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }
 
  // print the type of card
  Serial.println();
  Serial.print("Card type:         ");
  switch (SD.cardType()) {
    case CARD_NONE:
      Serial.println("NONE");
      break;
    case CARD_MMC:
      Serial.println("MMC");
      break;
    case CARD_SD:
      Serial.println("SD");
      break;
    case CARD_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }
 
  // print the type and size of the first FAT-type volume
//  uint32_t volumesize;
//  Serial.print("Volume type is:    FAT");
//  Serial.println(SDFS.usefatType(), DEC);
 
  Serial.print("Card size:  ");
  Serial.println((float)SD.cardSize()/1000);
 
  Serial.print("Total bytes: ");
  Serial.println(SD.totalBytes());
 
  Serial.print("Used bytes: ");
  Serial.println(SD.usedBytes());
 
  File dir =  SD.open("/test.txt", FILE_APPEND);  // Don't use FILE_WRITE unless you want to lose data
 
  dir.println("AD;FLKJASDFL;KJAS");     // also a print() function
  dir.close();

  delay(2000);
  Serial.println("Reading file");
  dir = SD.open("/test.txt");
     while(dir.available()){
        Serial.write(dir.read());
    }
  printDirectory(dir, 0);
  dir.close();
}
 
void loop(void) {
}
 
void printDirectory(File dir, int numTabs) {
  while (true) {
 
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.print(entry.size(), DEC);
      time_t lw = entry.getLastWrite();
      struct tm * tmstruct = localtime(&lw);
      Serial.printf("\tLAST WRITE: %d-%02d-%02d %02d:%02d:%02d\n", (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1, tmstruct->tm_mday, tmstruct->tm_hour, tmstruct->tm_min, tmstruct->tm_sec);
    }
    entry.close();
  }
}
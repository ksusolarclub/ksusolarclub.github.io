//Header File EncoderMonitor_H

//Define Encoder Monitor Header
#ifndef EncoderMonitor_H
#define EncoderMonitor_H 

//Declare Global Int
volatile int encoderPosition; //Define encoderPosition as a volatile int

void EncoderMonitorA()
{
  if (digitalRead(2) == digitalRead(3)) // If A == B
  {
      encoderPosition++; // Increment Encoder
  }
      else
      {
        encoderPosition--; // Decrment Encoder 
      }
}

void EncoderMonitorB()
{
  if (digitalRead(2) == digitalRead(3))// If A == B 
  {
    encoderPosition--; // Decremnt Encoder 
  }
    else
    {
      encoderPosition++; // Incremnt Encoder
    }
}

void EncoderInitialize()
{
 attachInterrupt( digitalPinToInterrupt(2), EncoderMonitorA, CHANGE );
 attachInterrupt( digitalPinToInterrupt(3), EncoderMonitorB, CHANGE );
} 
#endif

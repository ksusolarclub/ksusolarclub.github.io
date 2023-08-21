// Issues with ‘buttonState’ was not declared in this Scope
#ifndef ButtonDebounce_H
#define ButtonDebounce_H

enum buttonStates { buttonIdle, buttonWait, buttonLow } state;
unsigned long buttonTimer;

int ButtonInitialize()
{
  pinMode(4, INPUT);
  state = buttonIdle;
  pinMode (13, OUTPUT);
}

int ButtonNextState (int Press)
{
  switch (state)
  {
    case buttonIdle:
      if ( Press == LOW )
      {
        buttonTimer = millis();
        state = buttonWait;
        digitalWrite( 13, HIGH);
      }
      break;

    case buttonWait:
      if ( Press == HIGH )
      {
        state = buttonIdle;
        digitalWrite( 13, LOW );
      }
      else if ( millis() - buttonTimer >= 5)
      {
        state = buttonLow;
        digitalWrite ( 13, LOW );
        return 1;
      }
      break;
    case buttonLow:
      if ( Press == HIGH )
      {
        state = buttonIdle;
        return 2;
      }



  }
  return 0;
}
#endif

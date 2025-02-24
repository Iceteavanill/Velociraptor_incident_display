#include "STimer.h"

STimer::STimer(unsigned char _timerFunc, unsigned long _timeWait) : timerFunc{_timerFunc},
                                                                    timerWait{_timeWait}
{
  STimer::resetTimer();
  switch (timerFunc)
  {
  case STimer_State_TON:
    timerFuncPtr = &STimer::timerTon;
    break;

  case STimer_State_TOFF:
    timerFuncPtr = &STimer::timerToff;
    break;

  case STimer_State_TFF:
    timerFuncPtr = &STimer::timerTFF;
    break;

  default:
    timerFuncPtr = nullptr;
  }
}

void STimer::call()
{
  if (timerFuncPtr != nullptr)
  {
    (this->*timerFuncPtr)();
  }
  else
  {
    ; // oh no
  }
}

void STimer::resetTimer()
{
  milliTime = millis();
  call();
}

void STimer::timerTon()
{

  if (millis() - milliTime >= timerWait)
  {
    out = true;
  }
  else
  {
    out = false;
  }
}

void STimer::timerToff()
{

  if (millis() - milliTime <= timerWait)
  {
    out = true;
  }
  else
  {
    out = false;
  }
}

void STimer::timerTFF()
{

  if (millis() - milliTime >= timerWait)
  {
    out = !out;
    milliTime = millis();
  }
}
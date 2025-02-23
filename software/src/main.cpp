/*
This software was written by Iceteavanill for the Velociraptor Project.
It is provided as is with no liability for anything this software may or may not be the cause of.
For more information check the github readme.
This project uses the RTC library by Michael Miller TODO : add the rest!
*/

// libraries used
#include <Arduino.h>
#include <string.h>
#include <math.h>
#include <EEPROM.h>
#include <Wire.h>
#include <RtcDS1307.h>
#include <Button.h>
#include <PinChangeInterrupt.h>
// #include "avr8-stub.h"

// Timer Setup
#define USE_TIMER_2 true
#define USE_TIMER_1 true
#include <TimerInterrupt.h>

#include "setup.h"

RtcDS1307<TwoWire> Rtc(Wire); // setup for the RTC

// global variables
int brightnessoffset = 0;      // offset for led brightness (should be negative, typically it is zero)
int brightnessscaling = 4;     // scaling for brightness (should be between 1 and 10, typically it is 4)
int brighnessvalprocessed = 0; // sensor reading smoothed (average of 10 readings)

Button switchset{sSet}; // buttoninstance for the setswitch
Button switchinc{sInc}; // buttoninstance for the incswitch
Button switchdec{sDec}; // buttoninstance for the decswitch

byte errorcode = 0;        // contains a error code if one is triggered
byte errorIsDisplayed = 0; // contains the last error that was displayed

unsigned long timerton; // timer that can be used to create a simple timer no matter what time the RTC is

bool booncestatemachine = false; // used to execute functions only once when the state machine switchse state

// enumerator of the different states that the menu can be
enum systemstate
{
  state_noinit,                            // 00 : The state machine has not been set and sets some variables and steps to the next step
  state_fault,                             // 01 : A error has occured and the user has to be informed
  state_default,                           // 02 : the menu is in its default state of displaying passed days with unlocked switches
  state_locked,                            // 03 : the menu is in its default state of displaying passed days with locked switches
  state_setup,                             // 04 : the menu is in the setup mode and the setting can be selected that want to be changed
  state_setup_Brigtness_menu,              // 05 : setupmenu for setting the light menu
  state_setup_Brigtness_Displayvalue_menu, // 06 : displaying the values asociated with the Brighness setting
  state_setup_Brigtness_Displayvalue_exec, // 07 : displaying the values asociated with the Brighness setting
  state_setup_Brigtness_1,                 // 08 : step 1 in brightness calibration (determine zero offset)
  state_setup_Brigtness_2,                 // 09 : step 2 in brightness calibration (determin scaling factor)
  state_setup_time_menu,                   // 10 : setupmenu for setting the time
  state_setup_time_set,                    // 11 : setting the time
  state_setup_time_Display_menu,           // 12 : dislay the time menu option
  state_setup_time_Display_exec,           // 13 : dislay the time
  state_setup_resetcounter_menu,           // 14 : setupmenu for resetting the counter
  state_setup_resetcounter_reask           // 15 : Ask for confirmation to reset counter
};

enum errorstate
{
  error_RTCtime = B00000001,     // RTC time not valid
  error_RTCfatal = B00000010,    // RTC fatal error
  error_invalidtime = B00000100, // invalid time for the last incident
  error_brighness = B00001000,   // Brighness has invalid calibration values
  error_unrealistic = B00010000  // calibration values are unrealistic
};

systemstate statemachine; // system state

RtcDateTime rtctimecurrent = RtcDateTime(__DATE__, __TIME__); // init RTC time object with Compile time. This object contains the most recent time of the RTC
RtcDateTime rtctimeVfree = RtcDateTime(__DATE__, __TIME__);   // init RTC time object with Compile time. This object contains the last time a velociraptor incident has happened

/*
 RTC reference https://github.com/Makuna/Rtc/wiki
*/

// functions
void updatedisplay(const char *updateString, byte updateDots); // manages the displaying of characters
void setbrightness();                                          // smooths the sensor valus and set brightness for the display
void switchhandler();                                          // inputs from the switches and debounces them
void calcdisplaydefault(bool reload);                          // calculate the default display

#if DEBUG == 1 // this function is only used for debuging
void printDateTime(const RtcDateTime &dt);
#endif

// Setup
void setup()
{
  // debug_init();
  delay(500); // give everything time to power up

#if DEBUG == 1 // setup serial for debug
  Serial.begin(9600);
#endif

  debugln(F("------------------- Debuging enabled -------------------"));

  // pinmodes
  pinMode(dpClear, OUTPUT);
  pinMode(dpData, OUTPUT);
  pinMode(dpOe, OUTPUT);
  pinMode(dpClk, OUTPUT);
  pinMode(dpRclkU4, OUTPUT);
  pinMode(dpRclkU5, OUTPUT);
  pinMode(dpRclkU6, OUTPUT);
  pinMode(dpRclkU7, OUTPUT);
  pinMode(dplight, OUTPUT);

  pinMode(sSet, INPUT);
  pinMode(sInc, INPUT);
  pinMode(sDec, INPUT);

  attachPCINT(digitalPinToPCINT(sSet), switchhandler, CHANGE);
  attachPCINT(digitalPinToPCINT(sInc), switchhandler, CHANGE);
  attachPCINT(digitalPinToPCINT(sDec), switchhandler, CHANGE);

  // read EEPROMvalues for the brighness display and correct unrealistic values
  EEPROM.get(EEPROMadrOffset, brightnessoffset);

  if ((brightnessoffset < -500) || (brightnessoffset > 0))
  {
    debug(F("Brigness offset invalid It was corrected from : "));
    debugln(brightnessoffset);
    brightnessoffset = 0; // default offset
    errorcode |= error_brighness;
  }

  EEPROM.get(EEPROMadrScaling, brightnessscaling);

  if ((brightnessscaling > 10) or (brightnessscaling < 1))
  {
    debug(F("Brigness scaling invalid. It was corrected from : "));
    debugln(brightnessoffset);
    brightnessscaling = 4; // default scaling
    errorcode |= error_unrealistic;
  }

  debug(F("light offsetfactor is : "));
  debugln(brightnessoffset);
  debug(F("light scalingfactor is : "));
  debugln(brightnessscaling);

  // set the static IOs for and clear  the shift registers
  digitalWrite(dpOe, false);   // set outputenable false to enable the output (always the case)
  digitalWrite(dpClear, true); // set Clear to true to not clear the segments
  delay(1);
  digitalWrite(dpClear, false); // reset the shift register to a blank state
  delay(1);
  digitalWrite(dpClear, true);

  digitalWrite(dplight, 255); // set the display to full brighness (in case anything else does not work)

  Wire.begin();           // start I2C for RTC
  Wire.setTimeout(10000); // set timeout
  Rtc.Begin();            // start RTC

  if (!Rtc.GetIsRunning()) // check if RTC is running and set to run if not
  {
    Rtc.SetIsRunning(true);
    debugln(F("Rtc was not running"));
  }

  if (!Rtc.IsDateTimeValid()) // test if RTC is ok
  {
    errorcode |= error_RTCtime;
    debugln(F("RTC time not valid"));
    Rtc.SetDateTime(rtctimecurrent); // update the time to one thats valid
  }
  else // time is ok
  {
    debugln(F("RTC time is valid"));
    rtctimecurrent = Rtc.GetDateTime(); // sync time from RTC
  }

#if DEBUG == 1 // this function call is only used for debuging

  debug(F("rtctimecurrent : "));
  printDateTime(rtctimecurrent); // print system time

  debug(F("RTC : "));

  RtcDateTime timeforprint = Rtc.GetDateTime();

  printDateTime(timeforprint);

#endif

  if ((Rtc.LastError() != 0) || !Rtc.GetIsRunning())
  {
    errorcode |= error_RTCfatal;
    debug("RTC Error : ");
    debugln(Rtc.LastError());
  }

  EEPROM.get(EEPOMadrStarttime, rtctimeVfree); // get the time/date of the eeprom when the last velociraptor happened

  if (!rtctimeVfree.IsValid())
  { // check if the eeprom has valid time and set compile time if wrong
    debugln(F("A invalid Date/time was stored in the EEprom"));
    rtctimeVfree = rtctimecurrent;
    errorcode |= error_invalidtime;
  }
#if DEBUG == 1
  else
  {
    debug(F("Time since last incident : "));
    printDateTime(rtctimeVfree);
  }
#endif
  // set every segment to true to make a display test
  updatedisplay("8888", B00001111);
  delay(500); // give time to observe display defects

  ITimer2.init();
  ITimer2.attachInterruptInterval(calltime, setbrightness, 0);

  // Init completed
  debugln(F("------------------- Initialized -------------------"));
}

// loop
void loop()
{

  // State Machine :

#if DEBUG == 1
  static systemstate statemachinelast;
  if (statemachinelast != statemachine)
  {
    debug(F("The state changed from : "));
    debug(statemachinelast);
    debug(F(" to : "));
    debugln(statemachine);
    statemachinelast = statemachine;
  }
#endif

  /*  ------------------------------------------------------------
                      State Machine
      ------------------------------------------------------------ */
  switch (statemachine)
  {

    /*  ------------------------------------------------------------
                        Init
        ------------------------------------------------------------ */
  case state_noinit:
    debugln(F("init state was entered"));
    booncestatemachine = false;
    if (errorcode != 0) // go to error state if a arror is pendent
    {
      statemachine = state_fault;
    }
    else
    {
      statemachine = state_default;
    }
    break;

    /*  ------------------------------------------------------------
                        Default state with unlocked buttons
        ------------------------------------------------------------ */
  case state_default:

    if (!booncestatemachine)
    {
      booncestatemachine = true;
      calcdisplaydefault(true);
    }
    else
    {
      calcdisplaydefault(false); // the main function of this thingey
    }

    if (switchset.trigger())
    {
      booncestatemachine = false;
      statemachine = state_setup;
    }
    else if (switchdec.buttonStatus && switchinc.buttonStatus) // if both inc and dec are pressed at the same time, a timer of 5 seconds is startet
    {
      if (millis() - timerton >= 5000) // the two switches have to be held 5 seconds
      {
        booncestatemachine = false;
        timerton = millis();
        statemachine = state_locked;
      }
    }
    else // reset timer
    {
      timerton = millis();
    }
    break;

    /*  ------------------------------------------------------------
                        Default state with locked buttons
        ------------------------------------------------------------ */
  case state_locked:

    if (!booncestatemachine)
    {
      booncestatemachine = true;
      calcdisplaydefault(true);
    }
    else
    {
      calcdisplaydefault(false); // the main function of this thingey
    }

    if (switchdec.buttonStatus && switchinc.buttonStatus) // if both inc and dec are pressed at the same time, a timer of 5 seconds is startet
    {
      if (millis() - timerton >= 5000) // the two switches have to be held 5 seconds
      {
        booncestatemachine = false;
        timerton = millis();
        statemachine = state_default;
      }
    }
    else // reset timer
    {
      timerton = millis();
    }
    break;

    /*  ------------------------------------------------------------------------------------------------------------------------
                        Setup menu navigation
        ------------------------------------------------------------------------------------------------------------------------ */

    /*  ------------------------------------------------------------
                        Setup menu base
        ------------------------------------------------------------ */
  case state_setup:

    if (!booncestatemachine)
    {
      booncestatemachine = true;
      updatedisplay("set ", B00001111);
    }
    if (switchinc.trigger()) // navigate to the next menu
    {
      booncestatemachine = false;
      statemachine = state_setup_Brigtness_menu;
    }
    else if (switchdec.trigger()) // navigate to the previous menu
    {
      booncestatemachine = false;
      statemachine = state_setup_resetcounter_menu;
    }
    else if (switchset.trigger()) // navigate to the main menu
    {
      booncestatemachine = false;
      statemachine = state_default;
    }

    break;

    /*  ------------------------------------------------------------
                        Setup Brighness
        ------------------------------------------------------------ */
  case state_setup_Brigtness_menu:

    if (!booncestatemachine)
    {
      booncestatemachine = true;
      updatedisplay("ambl", B00001000);
    }

    if (switchinc.trigger()) // navigate to the next menu
    {
      booncestatemachine = false;
      statemachine = state_setup_Brigtness_Displayvalue_menu;
    }
    else if (switchdec.trigger()) // navigate to the previous menu
    {
      booncestatemachine = false;
      statemachine = state_setup;
    }
    else if (switchset.trigger()) // navigate to the sub menu
    {
      booncestatemachine = false;
      statemachine = state_setup_Brigtness_1;
    }

    break;

    /*  ------------------------------------------------------------
                        display Brighness values
        ------------------------------------------------------------ */
  case state_setup_Brigtness_Displayvalue_menu:

    if (!booncestatemachine)
    {
      booncestatemachine = true;
      updatedisplay("amb?", B00000100);
    }

    if (switchinc.trigger()) // navigate to the next menu
    {
      booncestatemachine = false;
      statemachine = state_setup_time_menu;
    }
    else if (switchdec.trigger()) // navigate to the previous menu
    {
      booncestatemachine = false;
      statemachine = state_setup_Brigtness_menu;
    }
    else if (switchset.trigger()) // navigate to the sub menu
    {
      booncestatemachine = false;
      statemachine = state_setup_Brigtness_Displayvalue_exec;
    }

    break;

    /*  ------------------------------------------------------------
                        Setup Time
        ------------------------------------------------------------ */
  case state_setup_time_menu:

    if (!booncestatemachine)
    {
      booncestatemachine = true;
      updatedisplay("time", B00000010);
    }

    if (switchinc.trigger()) // navigate to the next menu
    {
      booncestatemachine = false;
      statemachine = state_setup_time_Display_menu;
    }
    else if (switchdec.trigger()) // navigate to the previous menu
    {
      booncestatemachine = false;
      statemachine = state_setup_Brigtness_Displayvalue_menu;
    }
    else if (switchset.trigger()) // navigate to the sub menu
    {
      booncestatemachine = false;
      statemachine = state_setup_time_set;
    }

    break;

    /*  ------------------------------------------------------------
                        Display Time
        ------------------------------------------------------------ */
  case state_setup_time_Display_menu:

    if (booncestatemachine == false)
    {
      booncestatemachine = true;
      updatedisplay("tim?", B00000001);
    }

    if (switchinc.trigger()) // navigate to the next menu
    {
      booncestatemachine = false;
      statemachine = state_setup_resetcounter_menu;
    }
    else if (switchdec.trigger()) // navigate to the previous menu
    {
      booncestatemachine = false;
      statemachine = state_setup_time_menu;
    }
    else if (switchset.trigger()) // navigate to the sub menu
    {
      booncestatemachine = false;
      statemachine = state_setup_time_Display_exec;
    }

    break;

    /*  ------------------------------------------------------------
                        Reset time of last velociraptor incident
        ------------------------------------------------------------ */
  case state_setup_resetcounter_menu:

    if (booncestatemachine == false)
    {
      booncestatemachine = true;
      updatedisplay("res?", B00001001);
    }

    if (switchinc.trigger()) // navigate to the next menu
    {
      booncestatemachine = false;
      statemachine = state_setup;
    }
    else if (switchdec.trigger()) // navigate to the previous menu
    {
      booncestatemachine = false;
      statemachine = state_setup_time_Display_menu;
    }
    else if (switchset.trigger()) // navigate to the sub menu
    {
      booncestatemachine = false;
      statemachine = state_setup_resetcounter_reask;
    }

    break;

    /*  ------------------------------------------------------------------------------------------------------------------------
                        Different function steps
        ------------------------------------------------------------------------------------------------------------------------ */

    /*  ------------------------------------------------------------
                        setup brigness step 1 :  zero offset
        ------------------------------------------------------------ */
  case state_setup_Brigtness_1:

    if (booncestatemachine == false)
    {
      booncestatemachine = true;
      updatedisplay("stp1", B00001100);
    }

    if (switchset.trigger()) // do step 1 of brighness calibration
    {

      // on the last read write the new value
      brightnessoffset = -1 * brighnessvalprocessed;
      if ((brightnessoffset < -500) or (brightnessoffset > 0)) // check for realistic values
      {                                                        // unrealistic values

        debug(F("sensor reads : "));
        debugln(analogRead(aSiglight));
        debugln(F("offset not written : unrealistiv values -> setting offset to 0"));

        brightnessoffset = 0;

        errorcode = B00010000;

        booncestatemachine = false;
        statemachine = state_fault;
      }
      else
      { // realistic values

        EEPROM.put(EEPROMadrOffset, brightnessoffset);

        debug(F("sensor reads : "));
        debugln(analogRead(aSiglight));
        debug(F("Offset written : "));
        debugln(brightnessoffset);

        booncestatemachine = false;
        statemachine = state_setup_Brigtness_2;
      }
    }

    break;

    /*  ------------------------------------------------------------
                        setup brigness step 2 :  scaling
        ------------------------------------------------------------ */
  case state_setup_Brigtness_2:

    if (booncestatemachine == false)
    {
      booncestatemachine = true;
      updatedisplay("stp2", B00001111);
    }

    if (switchset.trigger()) // do step 2 of brighness calibration
    {

      brightnessscaling = ((brighnessvalprocessed + brightnessoffset) / 255) + 1;

      if ((brightnessscaling > 10) or (brightnessscaling < 1)) // check for realistic values
      {                                                        // unrealistic values

        debug(F("sensor reads : "));
        debugln(analogRead(aSiglight));
        debug(F("scaling not written : unrealistiv values -> scaling offset to 4"));

        brightnessoffset = 0;

        errorcode = B00010000;
        booncestatemachine = false;
        statemachine = state_fault;
      }
      else
      { // realistic values
        EEPROM.put(EEPROMadrScaling, brightnessscaling);

        debug(F("sensor reads : "));
        debugln(analogRead(aSiglight));
        debug(F("Scaling written : "));
        debugln(brightnessscaling);

        booncestatemachine = false;
        statemachine = state_setup_Brigtness_menu;
      }
    }
    break;

    /*  ------------------------------------------------------------
                        display Brighness values
        ------------------------------------------------------------ */
  case state_setup_Brigtness_Displayvalue_exec:
    static uint8_t whattodisplayBrighness;
    static uint8_t whattodisplayBrighnesslast;

    if (booncestatemachine == false)
    {
      whattodisplayBrighness = 0;
      whattodisplayBrighnesslast = 1;
      booncestatemachine = true;
    }

    if ((whattodisplayBrighness != whattodisplayBrighnesslast) || (whattodisplayBrighness == 0))
    {
      whattodisplayBrighnesslast = whattodisplayBrighness;
      char tmpString[] = "0000";
      byte tmpByte;
      switch (whattodisplayBrighness)
      {
      case 0: // display current light sensor
        sprintf(tmpString, "%4d", brighnessvalprocessed % 10000);
        tmpByte = B00001000;
        break;

      case 1: // display offset
        sprintf(tmpString, "%4d", brightnessoffset % 10000);
        tmpByte = B00000100;
        break;
      case 2: // display scaling
        sprintf(tmpString, "%4d", brightnessscaling % 10000);
        tmpByte = B00000010;
        break;
      default:
        whattodisplayBrighness = 0;
        whattodisplayBrighnesslast = 1;
        break;
      }
      updatedisplay(tmpString, tmpByte);
    }

    if (switchinc.trigger()) // go to next value
    {
      whattodisplayBrighness++;
    }
    else if (switchdec.trigger()) // go to last value
    {
      whattodisplayBrighness--;
      if (whattodisplayBrighness > 2)
      {
        whattodisplayBrighness = 2;
      }
    }
    if (switchset.trigger()) // go back to the menu
    {
      booncestatemachine = false;
      statemachine = state_setup_Brigtness_Displayvalue_menu;
    }

    break;

    /*  ------------------------------------------------------------
                        Setup Time
        ------------------------------------------------------------ */
  case state_setup_time_set:

    static uint8_t setupstep;
    static uint8_t setupsteplast;
    static uint8_t monthtemp; // local variable for the setting the months because it cant be set directly via incrementaion of seconds
    static uint16_t yeartemp; // local variable for the setting the Years because it cant be set directly via incrementaion of seconds

    if (booncestatemachine == false)
    {
      booncestatemachine = true;
      setupstep = 1;
      setupsteplast = 0;
      rtctimecurrent = Rtc.GetDateTime(); // at the start read the present time
      yeartemp = rtctimecurrent.Year();
      monthtemp = rtctimecurrent.Month();
    }

    if ((setupstep != setupsteplast) || (switchinc.trigger()) || (switchdec.trigger()))
    {
      setupsteplast = setupstep;
      char tmpString[] = "0000";
      byte tmpByte;

      switch (setupstep)
      {
      case 1: // set Year

        if (switchinc.trigger())
        {
          yeartemp++; // increment the temp year
        }
        else if (switchdec.trigger())
        {
          yeartemp--; // decrement the temp year
        }
        sprintf(tmpString, "%4d", yeartemp % 10000);
        tmpByte = B00001111;
        break;

      case 2: // set Month

        if (switchinc.trigger())
        {
          monthtemp++;        // increment the temp month
          if (monthtemp > 12) // month clamping
          {
            monthtemp = 1;
          }
        }
        else if (switchdec.trigger()) // month clamping
        {
          monthtemp--; // decrement the temp month
          if (monthtemp > 12)
          {
            monthtemp = 12;
          }
        }
        sprintf(tmpString, "%2dmm", monthtemp % 100);
        tmpByte = B00001100;
        break;

      case 3: // set Day

        if (switchinc.trigger())
        {
          rtctimecurrent += uint32_t(86400); // increment the time one day
        }
        else if (switchdec.trigger())
        {
          rtctimecurrent -= 86400; // decrement the time one day
        }
        sprintf(tmpString, "dd%2d", rtctimecurrent.Day() % 100);
        tmpByte = B00000011;
        break;

      case 4: // set Hour

        if (switchinc.trigger())
        {
          rtctimecurrent += uint32_t(3600); // increment the time one hour
        }
        else if (switchdec.trigger())
        {
          rtctimecurrent -= 3600; // decrement the time one hour
        }
        sprintf(tmpString, "%2dhh", rtctimecurrent.Hour() % 100);
        tmpByte = B00001100;
        break;

      case 5: // set minute

        if (switchinc.trigger())
        {
          rtctimecurrent += uint32_t(60); // increment the time one minute
        }
        else if (switchdec.trigger())
        {
          rtctimecurrent -= 60; // decrement the time one minute
        }
        sprintf(tmpString, "mm%2d", rtctimecurrent.Minute() % 100);
        tmpByte = B00000011;
        break;

      case 6: // set Seconds

        if (switchinc.trigger())
        {
          rtctimecurrent += uint32_t(1); // increment the time one Second
        }
        else if (switchdec.trigger())
        {
          rtctimecurrent -= 1; // decrement the time one Second
        }
        sprintf(tmpString, "ss%2d", rtctimecurrent.Second() % 100);
        tmpByte = B00001100;
        break;

      case 7:                             // write time and go to menu
      {                                   // explicit case here because placeholder crosses initialization
        RtcDateTime placeholder(yeartemp, // the
                                monthtemp,
                                rtctimecurrent.Day(),
                                rtctimecurrent.Hour(),
                                rtctimecurrent.Minute(),
                                rtctimecurrent.Second());

        if (placeholder.IsValid() && !(yeartemp <= 2000)) // check
        {
#if DEBUG == 1
          debug(F("The time was set to : "));
          printDateTime(rtctimecurrent);
#endif

          Rtc.SetDateTime(placeholder); // write the time to the RTC if the time is valid
          rtctimecurrent = placeholder;

          booncestatemachine = false;
          statemachine = state_default;
        }
        else
        {
          debugln(F("The time was not set due to it beeing not valid"));
          errorcode = B00010000;
          booncestatemachine = false;
          statemachine = state_fault;
        }
      }
      break;

      default:
        setupstep = 0;
        setupsteplast = 1;
        break;
      }

      updatedisplay(tmpString, tmpByte);
      debug(F("current Time setting step : "));
      debugln(setupstep);
    }

    if (switchset.trigger())
    {
      setupstep = setupstep + 1;
    }

    break;

    /*  ------------------------------------------------------------
                        display time
        ------------------------------------------------------------ */
  case state_setup_time_Display_exec:
    static uint8_t whattodisplayTime;
    static uint8_t whattodisplayTimelast;

    if (booncestatemachine == false)
    {
      whattodisplayTime = 0;
      whattodisplayTimelast = 1;
      booncestatemachine = true;
    }

    if ((whattodisplayTime != whattodisplayTimelast) || whattodisplayTime == 5)
    {
      rtctimecurrent = Rtc.GetDateTime();
      whattodisplayTimelast = whattodisplayTime;
      char tmpString[] = "    ";
      byte tmpByte;
      switch (whattodisplayTime)
      {
      case 0: // display Year
        sprintf(tmpString, "%4d", rtctimecurrent.Year() % 10000);
        tmpByte = B00001000;
        break;
      case 1: // display month
        sprintf(tmpString, "mm%2d", rtctimecurrent.Month() % 100);
        tmpByte = B00000010;
        break;
      case 2: // display day
        sprintf(tmpString, "dd%2d", rtctimecurrent.Day() % 100);
        tmpByte = B00000100;
        break;
      case 3: // display hour
        sprintf(tmpString, "hh%2d", rtctimecurrent.Hour() % 100);
        tmpByte = B00000010;
        break;
      case 4: // display minutes
        sprintf(tmpString, "mm%2d", rtctimecurrent.Minute() % 100);
        tmpByte = B00001100;
        break;
      case 5: // display seconds
        sprintf(tmpString, "ss%2d", rtctimecurrent.Second() % 100);
        tmpByte = B00001110;
        break;
      default:
        debugln(F("Time display error--------------------------------------"));
        whattodisplayTime = 0;
        whattodisplayTimelast = 1;
        break;
      }
      updatedisplay(tmpString, tmpByte);
    }

    if (switchinc.trigger()) // go to next value
    {
      whattodisplayTime++;
      if (whattodisplayTime > 5)
      {
        whattodisplayTime = 0;
      }
    }
    else if (switchdec.trigger()) // go to last value
    {
      whattodisplayTime--;
      if (whattodisplayTime > 5)
      {
        whattodisplayTime = 5;
      }
    }
    if (switchset.trigger()) // go back to the menu
    {
      statemachine = state_setup_time_Display_menu;
      booncestatemachine = false;
    }

    break;

    /*  ------------------------------------------------------------------------------------------------------------------------
                        Reset time : ask again
        ------------------------------------------------------------------------------------------------------------------------ */
  case state_setup_resetcounter_reask:

    if (booncestatemachine == false)
    {
      booncestatemachine = true;
      updatedisplay("sure", B00001111);
    }

    if (switchset.trigger()) // navigate to the parent menu
    {
      booncestatemachine = false;
      statemachine = state_setup;
    }
    else if (switchdec.buttonStatus && switchinc.buttonStatus) // if both inc and dec are pressed at the same time, a timer of 5 seconds is startet
    {

      if (millis() - timerton >= 5000) // the two switches have to be held 5 seconds
      {

        rtctimecurrent = Rtc.GetDateTime(); // sync both times
        rtctimeVfree = Rtc.GetDateTime();
        EEPROM.put(EEPOMadrStarttime, rtctimeVfree); // write time to EEPROM

        statemachine = state_default; // go back to menu after reset
        booncestatemachine = false;
        debugln(F("Time since last incident was reset to current time"));
      }
    }
    else // reset timer
    {
      timerton = millis();
    }

    break;

    /*  ------------------------------------------------------------
                        Error state
        ------------------------------------------------------------ */
  case state_fault:
  {
    /*
    Error states (1 is the least significant bit):
    1 : RTC time not valid
    2 : RTC fatal error
    3 : invalid time for the last incident
    4 : Brighness has invalid calibration values
    5 : calibration values are unrealistic
    */

    if (booncestatemachine == false)
    {
      debugln(F("Error state entered"));
      booncestatemachine = true;
      timerton = millis();
      updatedisplay("err ", B00000001);
      errorIsDisplayed = 0;
    }

    if ((millis() - timerton >= 5000))
    {
      if (!errorcode) // When every error has been displayed and the error is recoverable, go to normal operation. If not a reset is needed
      {
        debugln(F("All errors have been displayed"));
        statemachine = state_noinit;
      }
      debug(F("Ther is an error : "));
      debugln(errorcode);
      timerton = millis();
      byte errorToDisplay = errorIsDisplayed;
      errorIsDisplayed = 0;
      do
      {
        debugln(F("Looking for errors"));
        if (errorToDisplay == 0)
        { // start error display
          errorToDisplay = 1;
        }
        else if (errorToDisplay == 16) // loop around
        {
          errorToDisplay = 1;
        }
        else
        {
          errorToDisplay <<= 1; // normal shift to next
        }

        if ((errorToDisplay & errorcode) != 0)
        {
          debug(F("Error found : "));
          debugln(errorToDisplay);
          errorIsDisplayed = errorToDisplay;
        }
      } while ((errorIsDisplayed == 0) && errorcode); // loop through all errors

      debug("error to display : ");
      debugln(errorIsDisplayed);

      switch (errorIsDisplayed)
      {
      case error_RTCtime:
        updatedisplay("err1", B00000000);
        errorcode &= ~error_RTCtime; // clear error
        break;
      case error_RTCfatal:
        updatedisplay("err2", B00001111); // error not recoverable
        break;
      case error_invalidtime:
        updatedisplay("err3", B00000000);
        errorcode &= ~error_invalidtime; // clear error
        break;
      case error_brighness:
        updatedisplay("err4", B00000000);
        errorcode &= ~error_brighness; // clear error
        break;
      case error_unrealistic:
        updatedisplay("err5", B00000000);
        errorcode &= ~error_unrealistic; // clear error
        break;
      default:
        updatedisplay("err?", B00000000);
        debugln(F("Error code not found"));
        break;
      }
    }
  }
  break;

    /*  ------------------------------------------------------------
                        default (unknown state)
        ------------------------------------------------------------ */
  default:

    debug(F("unknown state : "));
    debugln(statemachine);
    booncestatemachine = false;
    statemachine = state_noinit;
    break;
  }
}

// Updating the Display with new data
void updatedisplay(const char *updateString, byte updateDots)
{
  /*

  The display segments are connected to the Shiftregister like this :
  bit 1 : DP <- least significant bit
  bit 2 : C
  bit 3 : B
  bit 4 : A
  bit 5 : F
  bit 6 : G
  bit 7 : D
  bit 8 : E

        a
      -----
     |     |
    f|     |b
     |     |
      -----
     |  g  |
    e|     |c
     |     |
      ----- o
        d    dp

  Bedgfabc0

  Based on these : https://github.com/dmadison/LED-Segment-ASCII/blob/master/Images/All%20Characters/7-Segment-ASCII-All.png
  */
  byte dataforshift = B00000000;

  // set shift and data pins to known values
  digitalWrite(dpClk, false);
  digitalWrite(dpData, false);

  debugln(F("Display update ------------"));
  debug(F("string written to registers : "));
  debugln(updateString);

  int len = 0;
  for (; updateString[len] != '\0'; len++)
    ;

  // debug(F("Strlen is: "));
  // debugln(len);
  len--;
  for (; len >= 0; len--)
  {

    switch (updateString[len])
    {
    case '0':
      dataforshift = B11011110;
      break;
    case '1':
      dataforshift = B00000110;
      break;
    case '2':
      dataforshift = B11101100;
      break;
    case '3':
      dataforshift = B01101110;
      break;
    case '4':
      dataforshift = B00110110;
      break;
    case '5':
      dataforshift = B01111010;
      break;
    case '6':
      dataforshift = B11111010;
      break;
    case '7':
      dataforshift = B00001110;
      break;
    case '8':
      dataforshift = B11111110;
      break;
    case '9':
      dataforshift = B01111110;
      break;
    case 'a':
      dataforshift = B10111110;
      break;
    case 'b':
      dataforshift = B11110010;
      break;
    case 'c':
      dataforshift = B11011000;
      break;
    case 'd':
      dataforshift = B11100110;
      break;
    case 'e':
      dataforshift = B11111000;
      break;
    case 'f':
      dataforshift = B10111000;
      break;
    case 'h':
      dataforshift = B10110110;
      break;
    case 'i':
      dataforshift = B10010000;
      break;
    case 'l':
      dataforshift = B11010000;
      break;
    case 'm':
      dataforshift = B10001010;
      break;
    case 'n':
      dataforshift = B10100010;
      break;
    case 'o':
      dataforshift = B11100010;
      break;
    case 'p':
      dataforshift = B10111100;
      break;
    case 'r':
      dataforshift = B10100000;
      break;
    case 's':
      dataforshift = B01111010;
      break;
    case 't':
      dataforshift = B11110000;
      break;
    case 'u':
      dataforshift = B11000010;
      break;
    case 'y':
      dataforshift = B01110110;
      break;
    case ' ':
      dataforshift = B00000000;
      break;
    case '?':
      dataforshift = B10101100;
      break;
    default:
      debugln(F("Unknown character"));
      dataforshift = B11110110;
      break;
    }

    if ((updateDots >> len) & 1)
    {
      dataforshift |= B00000001;
    }

    shiftOut(dpData, dpClk, MSBFIRST, dataforshift);

    //     debug(F("Byte "));
    //     debug(len);
    //     debug(F(" : "));
    // #if DEBUG == 1
    //     Serial.println(dataforshift, BIN);
    // #endif
  }

  // display the writen characters
  digitalWrite(dpRclkU4, true);
  digitalWrite(dpRclkU5, true);
  digitalWrite(dpRclkU6, true);
  digitalWrite(dpRclkU7, true);
  delay(1);
  digitalWrite(dpRclkU4, false);
  digitalWrite(dpRclkU5, false);
  digitalWrite(dpRclkU6, false);
  digitalWrite(dpRclkU7, false);
  delay(1);
  digitalWrite(dpClk, false);
  digitalWrite(dpData, false);

  debugln(F("display updated ------------"));
}

// adjust the brightness to ambient light setting
void setbrightness()
{
  // This part of the function reads the sensor and smooths the value

  static int sensvalues[10];
  static byte sensindex;

  sensvalues[sensindex] = analogRead(aSiglight);
  sensindex++;

  if (sensindex >= 10)
  {
    sensindex = 0;
  }

  for (byte i = 0; i < 10; i++)
  {
    brighnessvalprocessed = brighnessvalprocessed + sensvalues[i];
  }

  brighnessvalprocessed = brighnessvalprocessed / 10;

  // this function calculates the brighness value

  int brightnesstowrite;
  brightnesstowrite = brighnessvalprocessed;
  brightnesstowrite += brightnessoffset;
  brightnesstowrite /= brightnessscaling;

  if (brightnesstowrite > 255)
  {
    brightnesstowrite = 255;
  }

#if allowdisplayoff == 0
  else if (brightnesstowrite < 1)
  {
    brightnesstowrite = 1;
  }
#endif

  analogWrite(dplight, brightnesstowrite);

  // #if DEBUG == 1 // only print the Brighnessvalue every 5 seconds to not spam the Serial port
  //   static long timeforbrighness;
  //   if (millis() - timeforbrighness >= 5000)
  //   {
  //     timeforbrighness = millis();
  //     debug(F("brightness set to "));
  //     debugln(brightnesstowrite);
  //   }

  // #endif
}

void switchhandler()
{
  debug(F("Switch change detected :"));
  debugln(millis());
  switchset.scan();
  switchinc.scan();
  switchdec.scan();
}

#if DEBUG == 1 // this function is only used for debuging
void printDateTime(const RtcDateTime &dt)
{
  char datestring[20];
  debug(F("the time is : "));
  snprintf_P(datestring,
             countof(datestring),
             PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
             dt.Month(),
             dt.Day(),
             dt.Year(),
             dt.Hour(),
             dt.Minute(),
             dt.Second());
  debugln(datestring);
}
#endif

void calcdisplaydefault(bool reload) // calculate the default display
{

  static unsigned long secondsofday;  // seconds until it is midnight maximum is 86400
  static unsigned long lastmilisread; // timer to reduce RTC reads. the RTC gets read only
  static uint16_t dayspassed;         // the count of days passed
  static uint16_t dayspassedlast;     // count of days comparrison

  if (reload || (86370 <= (secondsofday + ((millis() - lastmilisread) / 1000))) || (30 >= (secondsofday + ((millis() - lastmilisread) / 1000)))) // when the time until midnight is within 30 seconds the RTC time gets read (I try to not read the RTC too much)
  {

    rtctimecurrent = Rtc.GetDateTime();
    dayspassed = rtctimecurrent.TotalDays() - rtctimeVfree.TotalDays();
    secondsofday = (long(rtctimecurrent.Hour()) * 3600) + (long(rtctimecurrent.Minute()) * 60) + long(rtctimecurrent.Second());
    lastmilisread = millis();

    debug(F("seconds of the day : "));
    debugln(secondsofday);
    debug(F("days passed rtc : "));
    debugln(rtctimecurrent.TotalDays());
    debug(F("days passed Vfree : "));
    debugln(rtctimeVfree.TotalDays());
  }

  if ((dayspassed != dayspassedlast) || reload)
  {
    reload = false;
    debugln(F("displaying days passed"));
    dayspassedlast = dayspassed;

    char tmpString[] = "    ";

    sprintf(tmpString, "%4d", dayspassed % 10000);
    updatedisplay(tmpString, statemachine == state_locked ? 0 : B00001000);
  }
}

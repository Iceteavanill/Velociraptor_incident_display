/*
This software was written by Iceteavanill for the Velociraptor incident Project.
It is provided as is with no liability for anything this software may or may not be the cause of.
For more information check the github readme.
This project uses the RTC library by Michael Miller, PinChangeInterrupt library by NicoHood and the TimerInterrupt library khoih-prog
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
#include <STimer.h>
#include <StateController.h>
// Timer Setup
#define USE_TIMER_2 true
#include <TimerInterrupt.h>

#include "setup.h"

// functions

// helperfunctions

/**
 * @brief Manages the displaying of characters
 *
 * The provided string and bytes are displayed on the 7-segment displays.
 * This function handles the conversion of characters to bit representation and shifting out of the bytes to the shift registers.
 * If a nullpointer is provided as a argument for the string, the function returns instantly.
 *
 * @param updateString const char * String to be displayed. Only the first 4 chars are interpreted
 *
 * @param updateDots byte Dots that should be displayed (interpreted bitwise)
 *
 */
void updatedisplay(const char *updateString, byte updateDots);

/**
 * @brief set brightness for the display
 *
 * Sets the brighness of the 7-segment displays.
 * The function calls getSensorValue, offsets and scales it according to calibration.
 * This function uses analogWrite. Therfore it occupies timer 1 of the atmega.
 *
 */
void setbrightness();

/**
 * @brief sets display according to the state
 *
 * This function is used as a action for a statechange.
 * It allows for automatic display of a default string and dots according to the ones stored in defaultDisplaysStr and defaultDisplaysByte.
 * It simply calls updatedisplay according to the provided step.
 *
 * @param step unsigned int of the current active step
 *
 */
void displayAccordingToState(unsigned int step);

/**
 * @brief get the light sensor value (smoothed)
 *
 * This function reads the current light level and keeps a moving average of the last 10 reading.
 *
 * @return int smoothed sensor value (average of 10 readings)
 *
 */
int getSensorValue();

/**
 * @brief calculates displays the days since last incident
 *
 * This function manages the displaying of the time since the last Velociraptor incident.
 * It also handles the transition of the new number (which happens at midnight)
 *
 */
void calcAndDisplayTimeSinceIncident();

/**
 * @brief This is a simple (and probably bad) implentation of sprintf
 *
 * This function was manly done because using the default Sprintf was using a lot of memory.
 * It takes a default string and writes a number on top of it(if ther is any number that overrides the string).
 * A zero is always written.
 * If a nullpointer is provided, it generates a error.
 *
 * @param baseStr const char * to a base String that gets overridden(dont provide a nullpointer)
 *
 * @param value uint16_t value that is printet over the defaultstring
 *
 * @param updateDots byte of dots that should be displayed (get passed through).
 *
 */
void sprintfToDisplay(const char *baseStr, uint16_t value, byte updateDots);

/**
 * @brief display the time of a specified RtcDateTime object
 *
 * This function provides a menu for displaying any RtcDateTime object.
 * Pressing the inc and dec button allows switching between the different steps.
 * It allows for realtime updating of the seconds.
 * This function needs the secondary state to be setup with 6 states (secondayState.reset(0, 6))
 *
 * @param dt const RtcDateTime& that should be displayed
 *
 */
void displayTime(const RtcDateTime &dt);

/**
 * @brief check for two switches to be held for 5 seconds
 *
 * Takes 2 button objects as input and checks if both have been held for 5 seconds simultaneously.
 * It that happens, the functions returns true. Any time else it returns false.
 * After a true output it resets its internal timer and continues from the beginning.
 *
 * @param button1 Button object that gets read for a true output.
 *
 * @param button2 Button object that gets read for a true output.
 *
 * @return bool that returns true once after the button has been held.
 *
 */
bool twoSwitchesHeldForTime(Button &button1, Button &button2);

/**
 * @brief set the time of a specified RtcDateTime object
 *
 * This function provides a menu to edit a RtcDateTime object.
 * Pressing inc and dec manipulates a the current value.
 * pressing set goes to the next step.
 * When the last step is completed, the function returns true anytime else, it returns false.
 * This is for saving the value somewhere (eeprom or RTC)
 *
 * @param timeToEdit RtcDateTime& object that gets edited
 *
 * @return bool true once after the last step has been completed
 *
 */
bool setCurrentTime(RtcDateTime &timeToEdit);

// ISR's

/**
 * @brief ISR for button events
 *
 * This function is called in case of any button input state change.
 * It does not know which button had a change.
 * It also rests the powerTimeout timer to set the CPU active again
 *
 */
void switchhandler();

/**
 * @brief handles periodic calling of setbrighness and wakup for counter update
 */
void periodicTasks();

// main state functions

/**
 * @brief main statemachine
 *
 * Contains the main statemachine. Handles the funcion calling of other funcions depending of current state.
 *
 */
void inline mainStatemachine();

/**
 * @brief resolve and display errors
 *
 * In case of a (or multiple) error(s), this function displays each one and if possible, clears em.
 *
 */
void inline resolveAndDisplayError();

/**
 * @brief display calibration step 1
 *
 * Actions that happen for step 1 of Sensor calibration (Sensor should be as dark as possible).
 *
 */
void inline brightnessCalibrationStp1();

/**
 * @brief display calibration step 2
 *
 * Actions that happen for step 2 of Sensor calibration (Sensor should be as bright as possible).
 *
 */
void inline brightnessCalibrationStp2();

/**
 * @brief display calibration step 2
 *
 * Actions that happen for displaying of the sensor calibration data and the current sensor value.
 *
 */
void inline displayCalibrationData(); // display calibration step 3

#if DEBUG == 1 // this function is only used for debuging
void printDateTime(const RtcDateTime &dt);
#endif

RtcDS1307<TwoWire> Rtc(Wire); // RTC instance

// global variables
int brightnessoffset = 0;  // offset for led brightness (should be negative, typically it is zero)
int brightnessscaling = 4; // scaling for brightness (should be between 1 and 10, typically it is 4)

unsigned long secondOfDayOfLastDisplayUpdate = 0; // seconds since the beginning of the day when the Displayed time was last updated
unsigned long millisOnDisplayUpdate = 0;          // millis that were read on display update

Button switchset{sSet}; // buttoninstance for the setswitch
Button switchinc{sInc}; // buttoninstance for the incswitch
Button switchdec{sDec}; // buttoninstance for the decswitch

byte errorcode = 0;        // contains a error code if one is triggered
byte errorIsDisplayed = 0; // contains the last error that was displayed

STimer tON{STimer_State_TON, 5000};           // timer that is used for simple delay during interactions (error display, wait for button push)
STimer powerTimeout{STimer_State_TON, 15000}; // timer to stay active for a while and then go to sleep

StateController mainState{SysState_noInit, &displayAccordingToState}; // main statemachine manager
StateController secondayState{0};                                     // secondary statemachine manager for nested states. Mainly display states

/*
 RTC reference https://github.com/Makuna/Rtc/wiki
*/

RtcDateTime rtctimecurrent = RtcDateTime(__DATE__, __TIME__); // init RTC time object with Compile time. This object contains the most recent time of the RTC
RtcDateTime rtctimeVfree = RtcDateTime(__DATE__, __TIME__);   // init RTC time object with Compile time. This object contains the last time a velociraptor incident has happened

// some error checking
// check if defaultDisplaysStr and defaultDisplaysByte is as big as SysState_NumOfTypes
static_assert(SysState_NumOfTypes == (sizeof(defaultDisplaysStr) / sizeof(defaultDisplaysStr[0])),
              "Number of states and number of default displays strings do not match");
static_assert(SysState_NumOfTypes == (sizeof(defaultDisplaysByte) / sizeof(defaultDisplaysByte[0])),
              "Number of states and number of default display byes do not match");
// check if error states are set correctly
static_assert((error_RTCtime |
               error_RTCfatal |
               error_invalidtime |
               error_brighness |
               error_unrealistic |
               error_Nullpointer) ==
                  (error_RTCtime +
                   error_RTCfatal +
                   error_invalidtime +
                   error_brighness +
                   error_unrealistic +
                   error_Nullpointer),
              "Error state numbering was not done correctly");

void setup()
{
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

  // set the IOs for and clear  the shift registers
  digitalWrite(dpOe, false);   // set outputenable false to enable the output (always the case)
  digitalWrite(dpClear, true); // set Clear to true to not clear the segments
  delay(1);
  digitalWrite(dpClear, false); // reset the shift register to a blank state
  delay(1);
  digitalWrite(dpClear, true);

  digitalWrite(dplight, 255); // set the display to full brighness (in case anything else does not work)

  Wire.begin();            // start I2C for RTC
  Wire.setTimeout(10000);  // set timeout
  Rtc.Begin();             // start RTC
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

  for (int i = 0; i < 9; i++)
  {
    getSensorValue(); // get the first sensor values to have a valid value
  }
  delay(500); // give time to observe display defects

  ITimer2.init();
  ITimer2.attachInterruptInterval(calltime, periodicTasks, 0); // start automatic brightness adjustment

  sleepMode(SLEEP_MODE_IDLE);

  // Init completed
  debugln(F("------------------- Initialized -------------------"));
}

void loop()
{
  mainStatemachine();
}

void mainStatemachine()
{
#if DEBUG == 1
  static unsigned int statemachinelast;
  if (statemachinelast != mainState.activeStep)
  {
    debug(F("The state changed from : "));
    debug(statemachinelast);
    debug(F(" to : "));
    debugln(mainState.activeStep);
    statemachinelast = mainState.activeStep;
  }
#endif

  switch (mainState.activeStep)
  {

  case SysState_noInit: // ------------------- init state -------------------
    debugln(F("init state was entered"));
    if (errorcode != 0) // go to error state if a error is pendent
    {
      mainState.nextStep(SysState_fault);
    }
    else
    {
      mainState.nextStep(SysState_idleUnlocked);
    }
    break;

  case SysState_idleUnlocked: // ------------------- default state -------------------
    if (mainState.doOnce())
    {
      powerTimeout.resetTimer();
      secondayState.reset(0, 3);
    }
    else
    {
      powerTimeout.call();
      calcAndDisplayTimeSinceIncident();
      if (powerTimeout.out)
      {
        debugln(F("Going to sleep"));
        enableSleep();
        startSleep();
        disableSleep();
      }
    }

    mainState.nextStepConditional(SysState_menu_Main, switchset.trigger());

    if (twoSwitchesHeldForTime(switchdec, switchinc))
    {
      mainState.nextStep(SysState_idleLocked);
    }

    break;

  case SysState_idleLocked: // ------------------- locked state -------------------
    if (mainState.doOnce())
    {
      powerTimeout.resetTimer();
      secondayState.reset(0, 3);
    }
    else
    {
      powerTimeout.call();
      calcAndDisplayTimeSinceIncident();
      if (powerTimeout.out)
      {
        debugln(F("Going to sleep"));
        enableSleep();
        startSleep();
        disableSleep();
      }
    }

    if (twoSwitchesHeldForTime(switchdec, switchinc))
    {
      mainState.nextStep(SysState_idleUnlocked);
    }

    break;

  case SysState_menu_Main: // ------------------- setup state -------------------
    mainState.nextStepConditional(SysState_menu_calibration, switchinc.trigger());
    mainState.nextStepConditional(SysState_menu_resetCounter, switchdec.trigger());
    mainState.nextStepConditional(SysState_idleUnlocked, switchset.trigger());
    break;

  case SysState_menu_calibration:
    mainState.nextStepConditional(SysState_menu_displayCalibrationData, switchinc.trigger());
    mainState.nextStepConditional(SysState_menu_Main, switchdec.trigger());
    mainState.nextStepConditional(SysState_setup_calibrationStp1, switchset.trigger());
    break;

  case SysState_menu_displayCalibrationData:
    mainState.nextStepConditional(SysState_menu_timeSet, switchinc.trigger());
    mainState.nextStepConditional(SysState_menu_calibration, switchdec.trigger());
    mainState.nextStepConditional(SysState_display_calibrationData, switchset.trigger());
    break;

  case SysState_menu_timeSet:
    mainState.nextStepConditional(SysState_menu_displayTime, switchinc.trigger());
    mainState.nextStepConditional(SysState_menu_displayCalibrationData, switchdec.trigger());
    mainState.nextStepConditional(SysState_setup_time, switchset.trigger());
    break;

  case SysState_menu_displayTime:
    mainState.nextStepConditional(SysState_menu_resetCounter, switchinc.trigger());
    mainState.nextStepConditional(SysState_menu_timeSet, switchdec.trigger());
    mainState.nextStepConditional(SysState_display_time, switchset.trigger());
    break;

#if DEBUG == 1 // dsiplay set Vtime

  case SysState_menu_resetCounter:
    mainState.nextStepConditional(SysState_menu_VtimeSet, switchinc.trigger());
    mainState.nextStepConditional(SysState_menu_displayTime, switchdec.trigger());
    mainState.nextStepConditional(SysState_setup_resetCounterConfirm, switchset.trigger());
    break;

  case SysState_menu_VtimeSet:

    mainState.nextStepConditional(SysState_menu_displayVTime, switchinc.trigger());
    mainState.nextStepConditional(SysState_menu_resetCounter, switchdec.trigger());
    mainState.nextStepConditional(SysState_setup_VTime, switchset.trigger());
    break;

  case SysState_menu_displayVTime:

    mainState.nextStepConditional(SysState_menu_Main, switchinc.trigger());
    mainState.nextStepConditional(SysState_menu_VtimeSet, switchdec.trigger());
    mainState.nextStepConditional(SysState_display_VTime, switchset.trigger());
    break;

#else
  case SysState_menu_resetCounter:
    mainState.nextStepConditional(SysState_menu_Main, switchinc.trigger());
    mainState.nextStepConditional(SysState_menu_displayTime, switchdec.trigger());
    mainState.nextStepConditional(SysState_setup_resetCounterConfirm, switchset.trigger());
    break;

#endif

  case SysState_setup_calibrationStp1:
    if (switchset.trigger()) // do step 1 of brighness calibration
    {
      brightnessCalibrationStp1();
      mainState.nextStep(SysState_setup_calibrationStp2);
    }
    else
    {
      getSensorValue(); // call function to cycle readings buffer
    }
    mainState.nextStepConditional(SysState_menu_calibration, switchdec.trigger() || switchinc.trigger()); // go back to the menu
    break;

  case SysState_setup_calibrationStp2:
    if (switchset.trigger()) // do step 2 of brighness calibration
    {
      brightnessCalibrationStp2();
      mainState.nextStep(SysState_menu_calibration);
    }
    else
    {
      getSensorValue(); // call function to cycle readings buffer
    }
    break;

  case SysState_display_calibrationData:
    if (mainState.doOnce())
    {
      secondayState.reset(0, 3);
    }
    displayCalibrationData();
    mainState.nextStepConditional(SysState_menu_displayCalibrationData, switchset.trigger());
    break;

  case SysState_setup_time:
    if (mainState.doOnce())
    {
      debugln(F("preparing time setup"));
      secondayState.reset(0, 8);
      rtctimecurrent = Rtc.GetDateTime();
    }
    if (setCurrentTime(rtctimecurrent))
    {
      Rtc.SetDateTime(rtctimecurrent);
      mainState.nextStep(SysState_idleUnlocked);
    }
    break;

  case SysState_setup_VTime:
    if (mainState.doOnce())
    {
      secondayState.reset(0, 8);
    }
    if (setCurrentTime(rtctimeVfree))
    {
      EEPROM.put(EEPOMadrStarttime, rtctimeVfree); // write time to EEPROM
      mainState.nextStep(SysState_idleUnlocked);
    }
    break;

  case SysState_display_time:
    if (mainState.doOnce())
    {
      debugln(F("preparing time display"));
      secondayState.reset(0, 6);
    }
    displayTime(Rtc.GetDateTime());
    mainState.nextStepConditional(SysState_menu_displayTime, switchset.trigger());
    break;

  case SysState_display_VTime:
    if (mainState.doOnce())
    {
      secondayState.reset(0, 6);
    }
    displayTime(rtctimeVfree);
    mainState.nextStepConditional(SysState_menu_displayTime, switchset.trigger());
    break;

  case SysState_setup_resetCounterConfirm:
    mainState.nextStepConditional(SysState_menu_Main, switchset.trigger());

    if (twoSwitchesHeldForTime(switchdec, switchinc))
    {
      rtctimecurrent = Rtc.GetDateTime(); // sync both times
      rtctimeVfree = Rtc.GetDateTime();
      EEPROM.put(EEPOMadrStarttime, rtctimeVfree); // write time to EEPROM

      mainState.nextStep(SysState_idleUnlocked);
      debugln(F("Time since last incident was reset to current time"));
    }
    break;

  case SysState_fault:
    if (mainState.doOnce())
    {
      debugln(F("Error state entered"));
      tON.resetTimer();
      errorIsDisplayed = 0;
    }
    resolveAndDisplayError();
    break;

  default:
    debug(F("unknown state : "));
    debugln(mainState.activeStep);
    mainState.nextStep(SysState_noInit);
    break;
  }
}

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

  if (updateString == NULL) // updatestring is Null (probably from default display) so return
  {
    debugln(F("updateString is NULL, returning"));
    return;
  }
  byte dataforshift = B00000000;

  // set shift and data pins to known values
  digitalWrite(dpClk, false);
  digitalWrite(dpData, false);

  // find end of string
  int len = 0;
  for (; updateString[len] != '\0'; len++)
    ;

  len--;
  for (; len >= 0; len--)
  { // loop through string from the back

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
    case 'v':
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

    if (updateDots & 1)
    {
      dataforshift |= B00000001;
    }
    updateDots >>= 1;

    shiftOut(dpData, dpClk, MSBFIRST, dataforshift);
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

void setbrightness()
{
  // calculate the brightness value
  int brightnesstowrite{(getSensorValue() + brightnessoffset) / brightnessscaling};

#if allowdisplayoff == 0
  if (brightnesstowrite < 1)
  {
    brightnesstowrite = 1;
  }
#endif

  analogWrite(dplight, brightnesstowrite > 255 ? 255 : brightnesstowrite); // set the brightness
}

void switchhandler()
{
  switchset.scan();
  switchinc.scan();
  switchdec.scan();

  if (powerTimeout.out) // when the timer ran out, the CPU should be sleeping
  {
    debugln(F("Waking up"));
    disableSleep();
    powerTimeout.resetTimer();
  }
}

void calcAndDisplayTimeSinceIncident()
{
  switch (secondayState.activeStep)
  {
  case 0:
  default:
    // device has switched from some other task to this. Read RTC and init timer for sleep reactivation
    rtctimecurrent = Rtc.GetDateTime();

    sprintfToDisplay("    ",
                     rtctimecurrent.TotalDays() - rtctimeVfree.TotalDays(),
                     mainState.activeStep == SysState_idleLocked ? 0 : 1);

    secondOfDayOfLastDisplayUpdate = long(rtctimecurrent.Second()) +
                                     long(rtctimecurrent.Minute()) * 60 +
                                     long(rtctimecurrent.Hour()) * 3600;

    millisOnDisplayUpdate = millis();

    secondayState.incrementStep(true);
    break;

  case 1: // wait until midnight
    secondayState.incrementStep(secondOfDayOfLastDisplayUpdate + ((millis() - millisOnDisplayUpdate) / 1000) >= 86400);

    break;

  case 2:                              // read RTC to sync time and check for time (should only happen once as long as it is not too inacurate)
    if (Rtc.GetDateTime().Hour() == 0) // if hour ticked over, its a new day ->
    {
      secondayState.nextStep(0);
    }
    break;
  }

  return;
}

void displayAccordingToState(unsigned int step)
{
  updatedisplay(defaultDisplaysStr[step], defaultDisplaysByte[step]);
}

void resolveAndDisplayError()
{
  tON.call();
  if ((tON.out))
  {
    if (!errorcode) // When every error has been displayed and the error is recoverable, go to normal operation, If no reset is needed
    {
      debugln(F("All errors have been displayed"));
      mainState.nextStep(SysState_noInit);
    }
    debug(F("Ther is an error : "));
    debugln(errorcode);
    tON.resetTimer();
    byte errorToDisplay = errorIsDisplayed;
    errorIsDisplayed = 0;
    do
    {
      debugln(F("Looking for errors"));
      if (errorToDisplay == 0)
      { // start error display
        errorToDisplay = 1;
      }
      else if (errorToDisplay == (error_NumOfTypes - 1)) // loop around
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
    case error_Nullpointer:
      updatedisplay("err6", B00000000);
      errorcode &= ~error_Nullpointer; // clear error
      break;
    default:
      updatedisplay("err?", B00000000);
      debugln(F("Error code not found"));
      break;
    }
  }
}

void brightnessCalibrationStp1()
{
  // on the last read write the new value
  brightnessoffset = -1 * getSensorValue();
  if ((brightnessoffset < -500) or (brightnessoffset > 0)) // check for realistic values
  {                                                        // unrealistic values
    brightnessoffset = 0;
    errorcode = B00010000;
    mainState.nextStep(SysState_fault);
  }
  else
  { // realistic values
    EEPROM.put(EEPROMadrOffset, brightnessoffset);
  }
}

void brightnessCalibrationStp2()
{

  brightnessscaling = ((getSensorValue() + brightnessoffset) / 255) + 1;

  if ((brightnessscaling > 10) or (brightnessscaling < 1)) // check for realistic values
  {                                                        // unrealistic values

    debug(F("sensor reads : "));
    debugln(analogRead(aSiglight));
    debug(F("scaling not written : unrealistiv values -> scaling offset to 4"));

    brightnessoffset = 0;

    errorcode = B00010000;
    mainState.nextStep(SysState_fault);
  }
  else
  { // realistic values
    EEPROM.put(EEPROMadrScaling, brightnessscaling);

    debug(F("sensor reads : "));
    debugln(analogRead(aSiglight));
    debug(F("Scaling written : "));
    debugln(brightnessscaling);
  }
}

void displayCalibrationData()
{
  if ((secondayState.doOnce()) || (secondayState.activeStep == 0)) // skip do once to keep getting new data
  {
    switch (secondayState.activeStep)
    {
    case 0: // display current light sensor
      sprintfToDisplay("    ", getSensorValue(), B0001);
      break;

    case 1: // display offset
      sprintfToDisplay("    ", brightnessoffset, B0010);
      break;
    case 2: // display scaling
      sprintfToDisplay("    ", brightnessscaling, B0100);
      break;

    default:
      secondayState.nextStep(0);
      break;
    }
  }
  secondayState.incrementStep(switchinc.trigger());
  secondayState.decrementStep(switchdec.trigger());
}

bool setCurrentTime(RtcDateTime &timeToEdit)
{
  static uint8_t monthtemp = 0; // local variable for the setting the months because it cant be set directly via incrementaion of seconds
  static uint16_t yeartemp = 0; // local variable for the setting the Years because it cant be set directly via incrementaion of seconds

  if (secondayState.doOnce() || switchinc.trigger() || switchdec.trigger())
  {
    debug(F("Timesetting Step : "));
    debugln(secondayState.activeStep);

    switch (secondayState.activeStep)
    {
    case 0: // init variables
      yeartemp = timeToEdit.Year();
      monthtemp = timeToEdit.Month();
      secondayState.nextStep(1);
      break;

    case 1: // set Year
      if (switchinc.buttonStatus)
      {
        yeartemp++; // increment the temp year
      }
      else if (switchdec.buttonStatus)
      {
        yeartemp--; // decrement the temp year
      }
      sprintfToDisplay("    ", yeartemp, B0000);
      break;

    case 2: // set Month
      if (switchinc.buttonStatus)
      {
        if (monthtemp >= 12)
        {
          monthtemp = 1;
        }
        else
        {
          monthtemp++;
        }
      }
      else if (switchdec.buttonStatus)
      {
        if (monthtemp <= 1)
        {
          monthtemp = 12;
        }
        else
        {
          monthtemp--;
        }
      }
      sprintfToDisplay("mm  ", monthtemp, B0001);
      break;

    case 3: // set Day
      if (switchinc.buttonStatus)
      {
        timeToEdit += uint32_t(86400); // increment the time one day
      }
      else if (switchdec.buttonStatus)
      {
        timeToEdit -= 86400; // decrement the time one day
      }
      sprintfToDisplay("dd  ", timeToEdit.Day(), B0010);
      break;

    case 4: // set Hour
      if (switchinc.buttonStatus)
      {
        timeToEdit += uint32_t(3600); // increment the time one hour
      }
      else if (switchdec.buttonStatus)
      {
        timeToEdit -= 3600; // decrement the time one hour
      }
      sprintfToDisplay("hh  ", timeToEdit.Hour(), B0100);
      break;

    case 5: // set minute
      if (switchinc.buttonStatus)
      {
        timeToEdit += uint32_t(60); // increment the time one minute
      }
      else if (switchdec.buttonStatus)
      {
        timeToEdit -= 60; // decrement the time one minute
      }
      sprintfToDisplay("mm  ", timeToEdit.Minute(), B1000);
      break;

    case 6: // set Seconds
      if (switchinc.buttonStatus)
      {
        timeToEdit += uint32_t(1); // increment the time one Second
      }
      else if (switchdec.buttonStatus)
      {
        timeToEdit -= 1; // decrement the time one Second
      }
      sprintfToDisplay("ss  ", timeToEdit.Second(), B0011);
      break;

    case 7: // write time and go to menu
    {
      RtcDateTime placeholder(yeartemp,
                              monthtemp,
                              timeToEdit.Day(),
                              timeToEdit.Hour(),
                              timeToEdit.Minute(),
                              timeToEdit.Second());

      if (placeholder.IsValid() && !(yeartemp <= 2000)) // check
      {
#if DEBUG == 1
        debug(F("The time was set to : "));
        printDateTime(timeToEdit);
#endif

        timeToEdit = placeholder;
        return true; // time setting is completed.
      }
      else
      {
        debugln(F("The time was not set due to it beeing not valid"));
        errorcode = B00010000;
        mainState.nextStep(SysState_fault);
      }
      break;
    }
    default:
      debugln(F("unknown step in time setting"));
      secondayState.nextStep(0);
      break;
    }

    debug(F("current Time setting step : "));
    debugln(secondayState.activeStep);
  }

  secondayState.incrementStep(switchset.trigger());
  return false; // always return false until time setting is completed
}

int getSensorValue()
{
  static int sensvalues[10];
  static byte sensindex = 0;
  sensvalues[sensindex] = analogRead(aSiglight);
  sensindex++;

  if (sensindex >= 10)
  {
    sensindex = 0;
  }
  int smoothedvalue = 0;
  for (byte i = 0; i < 10; i++)
  {
    smoothedvalue += sensvalues[i];
  }
  smoothedvalue /= 10;
  return smoothedvalue > 1024 ? 1024 : smoothedvalue; // clamp value
}

void sprintfToDisplay(const char *baseStr, uint16_t value, byte updateDots)
{
  debugln(F("SPRINTF Called"));

  char buffer[] = {'\0', '\0', '\0', '\0', '\0'};
  unsigned int i = 0;
  if (baseStr == NULL)
  {
    debugln(F("baseStr is NULL"));
    errorcode |= error_Nullpointer;
    mainState.nextStep(SysState_fault);
    return;
  }
  while (baseStr[3 - i] != '\0')
  {
    buffer[3 - i] = baseStr[3 - i];
    if (value != 0 || i == 0)
    { // if the value is 0, the first should still be written
      buffer[3 - i] = value % 10 + '0';
      value /= 10;
    }
    i++;
  }

  updatedisplay(buffer, updateDots);
}

void displayTime(const RtcDateTime &dt)
{
  if ((secondayState.doOnce()) || secondayState.activeStep == 5) // bypass do once to keep getting new data because seconds may change
  {
    switch (secondayState.activeStep)
    {
    case 0: // display Year
      sprintfToDisplay("    ", dt.Year(), B0000);
      break;

    case 1: // display month
      sprintfToDisplay("mm  ", dt.Month(), B0001);
      break;

    case 2: // display day
      sprintfToDisplay("dd  ", dt.Day(), B0010);
      break;

    case 3: // display hour
      sprintfToDisplay("hh  ", dt.Hour(), B0100);
      break;

    case 4: // display minutes
      sprintfToDisplay("mm  ", dt.Minute(), B1000);
      break;

    case 5: // display seconds
      sprintfToDisplay("ss  ", dt.Second(), B0011);
      break;
    default:
      debugln(F("Time display error--------------------------------------"));
      secondayState.nextStep(0);
      break;
    }
  }
  secondayState.incrementStep(switchinc.trigger());
  secondayState.decrementStep(switchdec.trigger());
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

bool twoSwitchesHeldForTime(Button &button1, Button &button2)
{
  static STimer holdTimer{STimer_State_TON, 5000};
  if (button1.buttonStatus && button2.buttonStatus)
  {
    holdTimer.call();
    if (holdTimer.out)
    {
      holdTimer.resetTimer();
      return true;
    }
    return false;
  }
  else // reset timer
  {
    holdTimer.resetTimer();
    return false;
  }
}

void periodicTasks()
{
  setbrightness();

  if (powerTimeout.out) // when the timer ran out, the CPU should be sleeping
  {
    if (secondOfDayOfLastDisplayUpdate + ((millis() - millisOnDisplayUpdate) / 1000) >= 86395) // check if the time is close to midnight(within 5 seconds)
    {
      debugln(F("Waking up"));
      disableSleep();
      powerTimeout.resetTimer();
    }
  }
}
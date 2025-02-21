/*
This software was written by Iceteavanill for the Velociraptor Project.
It is provided as is with no liability for anything this software may or may not be the cause of.
For more information check the github readme.
This project uses the RTC library by Michael Miller TODO : add the rest!
*/

//libraries used
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <RtcDS1307.h>
#include <Button.h>
#include <PinChangeInterrupt.h>

//Timer Setup
#define USE_TIMER_2 true
#include <TimerInterrupt.h>

#include "setup.h"

RtcDS1307<TwoWire> Rtc(Wire); //setup for the RTC

//global variables
char displaychar[] = "err "; //initialize string with err in case something goes wrong. This array is for storing the characters that should be displayed. The characters are in order left to right(0 is shown by D2 and 3 is shown by D5)
bool displaydot[4] = {false,false,false,false}; // these represent the dots on the 7 segment display (folows the same logic as displaychar)

int brightnessoffset; //offset for led brightness (should be negative, typically it is zero)
int brightnessscaling; //scaling for brightness (should be between 1 and 10, typically it is 4)
int brighnessvalprocessed; //sensor reading smoothed (average of 10 readings)

Button switchset{sSet}; //buttoninstance for the setswitch
Button switchinc{sInc}; //buttoninstance for the incswitch
Button switchdec{sDec}; //buttoninstance for the decswitch

bool buttonpressed = false; // true if any switch is pressed
bool switchtrig; // used for only using an edge of a button


bool erroroccurednorecovery = false; // true if an error has occured that cant be recovered This flag needs a powercycle to reset
byte errorcode = 0; //contains a error code if one is triggered

unsigned long timerton; //timer that can be used to create a simple timer no matter what time the RTC is

bool booncestatemachine; // used to execute functions only once when the state machine switchse state

//enumerator of the different sates that the menu can be
enum systemstate 
{
  state_noinit,                             // 00 : The state machine has not been set and sets some variables and steps to the next step
  state_fault,                              // 01 : A error has occured and the user has to be informed
  state_default,                            // 02 : the menu is in its default state of displaying passed days with unlocked switches
  state_locked,                             // 03 : the menu is in its default state of displaying passed days with locked switches
  state_setup,                              // 04 : the menu is in the setup mode and the setting can be selected that want to be changed
  state_setup_Brigtness_menu,               // 05 : setupmenu for setting the light menu
  state_setup_Brigtness_Displayvalue_menu,  // 06 : displaying the values asociated with the Brighness setting
  state_setup_Brigtness_Displayvalue_exec,  // 07 : displaying the values asociated with the Brighness setting
  state_setup_Brigtness_1,                  // 08 : step 1 in brightness calibration (determine zero offset)
  state_setup_Brigtness_2,                  // 09 : step 2 in brightness calibration (determin scaling factor)
  state_setup_time_menu,                    // 10 : setupmenu for setting the time
  state_setup_time_set,                     // 11 : setting the time
  state_setup_time_Display_menu,            // 12 : dislay the time menu option
  state_setup_time_Display_exec,            // 13 : dislay the time
  state_setup_resetcounter_menu,            // 14 : setupmenu for resetting the counter
  state_setup_resetcounter_reask            // 15 : Ask for confirmation to reset counter
};

systemstate statemachine;//system state

RtcDateTime rtctimecurrent = RtcDateTime(__DATE__, __TIME__); //init RTC time object with Compile time. This object contains the most recent time of the RTC
RtcDateTime rtctimeVfree = RtcDateTime(__DATE__, __TIME__); //init RTC time object with Compile time. This object contains the last time a velociraptor incident has happened

/*
 RTC reference https://github.com/Makuna/Rtc/wiki
*/

//functions
void updatedisplay();//manages the displaying of characters
void setbrightness();//smooths the sensor valus and set brightness for the display
void switchhandler();//inputs from the switches and debounces them
void calcdisplaydefault(bool reload); //calculate the default display

#if DEBUG == 1 //this function is only used for debuging
  void printDateTime(const RtcDateTime& dt);
#endif

//Setup
void setup() 
{

  delay(500); //give everything time to power up

  #if DEBUG == 1 //setup serial for debug
    Serial.begin(9600);
  #endif

  debugln("------------------- Debuging enabled -------------------");

  //pinmodes
  pinMode(dpClear,OUTPUT);
  pinMode(dpData,OUTPUT);
  pinMode(dpOe,OUTPUT);
  pinMode(dpClk,OUTPUT);
  pinMode(dpRclkU4,OUTPUT);
  pinMode(dpRclkU5,OUTPUT);
  pinMode(dpRclkU6,OUTPUT);
  pinMode(dpRclkU7,OUTPUT);
  pinMode(dplight,OUTPUT);

  pinMode(sSet,INPUT);
  pinMode(sInc,INPUT);
  pinMode(sDec,INPUT);
  
  attachPCINT(digitalPinToPCINT(sSet), switchhandler, CHANGE);
  attachPCINT(digitalPinToPCINT(sInc), switchhandler, CHANGE);
  attachPCINT(digitalPinToPCINT(sDec), switchhandler, CHANGE);

  //read EEPROMvalues for the brighness display and correct unrealistic values
  EEPROM.get(EEPROMadrOffset,brightnessoffset);

  if((brightnessoffset < -500) or (brightnessoffset > 0))
  {
    debug("Brigness offset invalid It was corrected from : ");
    debugln(brightnessoffset);
    brightnessoffset = 0; //default offset
    errorcode = errorcode | B00001000;

  }

  EEPROM.get(EEPROMadrScaling,brightnessscaling);

  if((brightnessscaling > 10) or (brightnessscaling < 1))
  {
    debug("Brigness scaling invalid. It was corrected from : ");
    debugln(brightnessoffset);
    brightnessscaling = 4; //default scaling
    errorcode = errorcode | B00001000;

  }
    
  debug("light offsetfactor is : ");
  debugln(brightnessoffset);
  debug("light scalingfactor is : ");
  debugln(brightnessscaling);

  //set the static IOs for and clear  the shift registers 
  digitalWrite(dpOe,false); //set outputenable false to enable the output (always the case)
  digitalWrite(dpClear,true); //set Clear to true to not clear the segments
  delay(1);
  digitalWrite(dpClear,false); //reset the shift register to a blank state
  delay(1);
  digitalWrite(dpClear,true); 

  digitalWrite(dplight,255);//set the display to full brighness (in case anything else does not work)

  Wire.begin();//start I2C for RTC
  Wire.setTimeout(10000);//set timeout
  Rtc.Begin();//start RTC

  if (!Rtc.GetIsRunning())//check if RTC is running and set to run if not
  {
    Rtc.SetIsRunning(true);
    debugln("Rtc was not running");
  }

  if(!Rtc.IsDateTimeValid()) //test if RTC is ok
  {
    errorcode = errorcode | B00000001;
    debugln("RTC time not valid");
    Rtc.SetDateTime(rtctimecurrent);//update the time to one thats valid
  }
  else //time is ok
  {
    debugln("RTC time is valid");
    rtctimecurrent = Rtc.GetDateTime();//sync time from RTC
  }
  
  #if DEBUG == 1 //this function call is only used for debuging

    debug("rtctimecurrent : "); 
    printDateTime(rtctimecurrent); // print system time

    debug("RTC : ");

    RtcDateTime timeforprint = Rtc.GetDateTime();

    printDateTime(timeforprint);

  #endif
  
  if ((Rtc.LastError() != 0) || !Rtc.GetIsRunning())
  {
    errorcode = errorcode | B00000010;
    erroroccurednorecovery = true;
    debug("RTC Error : ");
    debugln(Rtc.LastError());
  }

  EEPROM.get(EEPOMadrStarttime,rtctimeVfree);// get the time/date of the eeprom when the last velociraptor happened

  if(!rtctimeVfree.IsValid())
  { // check if the eeprom has valid time and set compile time if wrong
    debugln("A invalid Date/time was stored in the EEprom");
    rtctimeVfree = rtctimecurrent;
    errorcode = errorcode | B00000100;
  }
  else
  {
    ;
    #if DEBUG == 1
      debug("Time since last incident : ");
      printDateTime(rtctimeVfree);
    #endif
  }

  //set every segment to true to make a display test
  displaychar[0] = '8';
  displaychar[1] = '8';
  displaychar[2] = '8';
  displaychar[3] = '8';
  displaydot[0] = true;
  displaydot[1] = true;
  displaydot[2] = true;
  displaydot[3] = true;
  updatedisplay();
  delay(500);

  ITimer2.init();
  ITimer2.attachInterruptInterval(calltime, setbrightness, 0);

  //Init completed
  debugln("------------------- Initialized -------------------");
}

//loop
void loop()
{

// State Machine : 

  #if DEBUG == 1
    static systemstate statemachinelast;
    if(statemachinelast != statemachine)
    {
      debug("The state changed from : ");
      debug(statemachinelast);
      debug(" to : ");
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
    debugln("init state was entered");
    switchtrig = false;
    booncestatemachine = false;
    if((errorcode != 0) || erroroccurednorecovery)//go to error state if a arror is pendent
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

    if(!booncestatemachine)
    {
      displaydot[0] = false;
      displaydot[1] = false;
      displaydot[2] = false;
      displaydot[3] = true;
      booncestatemachine = true;
      calcdisplaydefault(true);

    }
    else
    {
      calcdisplaydefault(false);//the main function of this thingey
    }

    if (!buttonpressed && switchtrig)//wait for switches to be unpressed
    {
      switchtrig = false;
    }
    else if(switchset.buttonStatus && !switchtrig)
    {
      switchtrig = true;
      booncestatemachine = false;
      statemachine = state_setup;
    }
    else if (switchdec.buttonStatus && switchinc.buttonStatus && !switchtrig)//if both inc and dec are pressed at the same time, a timer of 5 seconds is startet
    {

      if(millis() - timerton >= 5000) // the two switches have to be held 5 seconds
      {

      switchtrig = true;
      booncestatemachine = false;
      statemachine = state_locked;

      }     
    }
    else //reset timer
    {
      timerton = millis();
    }
    break;

/*  ------------------------------------------------------------
                    Default state with locked buttons
    ------------------------------------------------------------ */
  case state_locked:

    if(!booncestatemachine)
    {      
      displaydot[0] = false;
      displaydot[1] = false;
      displaydot[2] = false;
      displaydot[3] = false;
      booncestatemachine = true;
      calcdisplaydefault(true);

    }
    else
    {
      calcdisplaydefault(false);//the main function of this thingey
    }


    if (!buttonpressed && switchtrig)//wait for switches to be unpressed
    {
      switchtrig = false;
    }
    else if (switchdec.buttonStatus && switchinc.buttonStatus && !switchtrig)//if both inc and dec are pressed at the same time, a timer of 5 seconds is startet
    {
      if(millis() - timerton >= 5000) // the two switches have to be held 5 seconds
      {
        switchtrig = true;
        booncestatemachine = false;
        statemachine = state_default;
      }    
    }
    else //reset timer
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

    if(!booncestatemachine)
    {

      booncestatemachine = true;
      displaychar[0] = 's';
      displaychar[1] = 'e';
      displaychar[2] = 't';
      displaychar[3] = ' ';
      displaydot[0] = true;
      displaydot[1] = true;
      displaydot[2] = true;
      displaydot[3] = true;
      updatedisplay();
    }

    if (!buttonpressed && switchtrig)//wait for buttons to be unpressed
    {
      switchtrig = false;
    }
    else if(switchinc.buttonStatus && !switchtrig)//navigate to the next menu
    {
      switchtrig = true;
      booncestatemachine = false;
      statemachine = state_setup_Brigtness_menu;
    }
    else if (switchdec.buttonStatus && !switchtrig)//navigate to the previous menu
    {
      switchtrig = true;
      booncestatemachine = false;
      statemachine = state_setup_resetcounter_menu;

    }
    else if (switchset.buttonStatus && !switchtrig)//navigate to the main menu
    {
      switchtrig = true;
      booncestatemachine = false;
      statemachine = state_default;

    }
    
    break;
    

/*  ------------------------------------------------------------
                    Setup Brighness
    ------------------------------------------------------------ */
  case state_setup_Brigtness_menu:

    if(!booncestatemachine)
    {
      booncestatemachine = true;
      displaychar[0] = 'a';
      displaychar[1] = 'm';
      displaychar[2] = 'b';
      displaychar[3] = 'l';
      displaydot[0] = true;
      displaydot[1] = false;
      displaydot[2] = false;
      displaydot[3] = false;
      updatedisplay();
    }

    if (!buttonpressed && switchtrig)//wait for switches to be unpressed
    {
      switchtrig = false;
    }
    else if(switchinc.buttonStatus && !switchtrig)//navigate to the next menu
    {
      switchtrig = true;
      booncestatemachine = false;
      statemachine = state_setup_Brigtness_Displayvalue_menu;
    }
    else if (switchdec.buttonStatus && !switchtrig)//navigate to the previous menu
    {
      switchtrig = true;
      booncestatemachine = false;
      statemachine = state_setup;
    }
    else if (switchset.buttonStatus && !switchtrig)//navigate to the sub menu
    {
      switchtrig = true;
      booncestatemachine = false;
      statemachine = state_setup_Brigtness_1;
    }
    
    break;

/*  ------------------------------------------------------------
                    display Brighness values
    ------------------------------------------------------------ */
  case state_setup_Brigtness_Displayvalue_menu:

    if(!booncestatemachine)
    {
      booncestatemachine = true;
      displaychar[0] = 'a';
      displaychar[1] = 'm';
      displaychar[2] = 'b';
      displaychar[3] = '?';
      displaydot[0] = false;
      displaydot[1] = true;
      displaydot[2] = false;
      displaydot[3] = false;
      updatedisplay();
    }

    if (!buttonpressed && switchtrig)//wait for switches to be unpressed
    {
      switchtrig = false;
    }
    else if(switchinc.buttonStatus && !switchtrig)//navigate to the next menu
    {
      switchtrig = true;
      booncestatemachine = false;
      statemachine = state_setup_time_menu;
    }
    else if (switchdec.buttonStatus && !switchtrig)//navigate to the previous menu
    {
      switchtrig = true;
      booncestatemachine = false;
      statemachine = state_setup_Brigtness_menu;
    }
    else if (switchset.buttonStatus && !switchtrig)//navigate to the sub menu
    {
      switchtrig = true;
      booncestatemachine = false;
      statemachine = state_setup_Brigtness_Displayvalue_exec;
    }
    
    break;

/*  ------------------------------------------------------------
                    Setup Time
    ------------------------------------------------------------ */
  case state_setup_time_menu:

    if(!booncestatemachine)
    {
      booncestatemachine = true;
      displaychar[0] = 't';
      displaychar[1] = 'i';
      displaychar[2] = 'm';
      displaychar[3] = 'e';
      displaydot[0] = false;
      displaydot[1] = false;
      displaydot[2] = true;
      displaydot[3] = false;
      updatedisplay();
    }

    if (!buttonpressed && switchtrig)//wait for switches to be unpressed
    {
      switchtrig = false;
    }
    else if(switchinc.buttonStatus && !switchtrig)//navigate to the next menu
    {
      switchtrig = true;
      booncestatemachine = false;
      statemachine = state_setup_time_Display_menu;
    }
    else if (switchdec.buttonStatus && !switchtrig)//navigate to the previous menu
    {
      switchtrig = true;
      booncestatemachine = false;
      statemachine = state_setup_Brigtness_Displayvalue_menu;
    }
    else if (switchset.buttonStatus && !switchtrig)//navigate to the sub menu
    {
      switchtrig = true;
      booncestatemachine = false;
      statemachine = state_setup_time_set;
    }
    
    break;

/*  ------------------------------------------------------------
                    Display Time
    ------------------------------------------------------------ */
  case state_setup_time_Display_menu:

    if(booncestatemachine == false)
    {
      booncestatemachine = true;
      displaychar[0] = 't';
      displaychar[1] = 'i';
      displaychar[2] = 'm';
      displaychar[3] = '?';
      displaydot[0] = false;
      displaydot[1] = false;
      displaydot[2] = false;
      displaydot[3] = true;
      updatedisplay();
    }

    if (!buttonpressed && switchtrig)//wait for switches to be unpressed
    {
      switchtrig = false;
    }
    else if(switchinc.buttonStatus && !switchtrig)//navigate to the next menu
    {
      switchtrig = true;
      booncestatemachine = false;
      statemachine = state_setup_resetcounter_menu;
    }
    else if (switchdec.buttonStatus && !switchtrig)//navigate to the previous menu
    {
      switchtrig = true;
      booncestatemachine = false;
      statemachine = state_setup_time_menu;
    }
    else if (switchset.buttonStatus && !switchtrig)//navigate to the sub menu
    {
      switchtrig = true;
      booncestatemachine = false;
      statemachine = state_setup_time_Display_exec;
    }
    
    break;    

/*  ------------------------------------------------------------
                    Reset time of last velociraptor incident
    ------------------------------------------------------------ */
  case state_setup_resetcounter_menu:

     if(booncestatemachine == false)
    {
      booncestatemachine = true;
      displaychar[0] = 'r';
      displaychar[1] = 'e';
      displaychar[2] = 's';
      displaychar[3] = '?';
      displaydot[0] = true;
      displaydot[1] = false;
      displaydot[2] = false;
      displaydot[3] = true;
      updatedisplay();
    }

    if (!buttonpressed && switchtrig)//wait for switches to be unpressed
    {
      switchtrig = false;
    }

    if(switchinc.buttonStatus && !switchtrig)//navigate to the next menu
    {
      switchtrig = true;
      booncestatemachine = false;
      statemachine = state_setup;
    }
    else if (switchdec.buttonStatus && !switchtrig)//navigate to the previous menu
    {
      switchtrig = true;
      booncestatemachine = false;
      statemachine = state_setup_time_Display_menu;
    }
    else if (switchset.buttonStatus && !switchtrig)//navigate to the sub menu
    {
      switchtrig = true;
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

    if(booncestatemachine == false)
    {
      booncestatemachine = true;
      displaychar[0] = 's';
      displaychar[1] = 't';
      displaychar[2] = 'p';
      displaychar[3] = '1';
      displaydot[0] = true;
      displaydot[1] = true;
      displaydot[2] = false;
      displaydot[3] = false;
      updatedisplay();
    }

    if (!buttonpressed && switchtrig)//wait for switches to be unpressed
    {
      switchtrig = false;
    }
    else if (switchset.buttonStatus && !switchtrig)// do step 1 of brighness calibration
    {

      //on the last read write the new value 
      brightnessoffset = -1 * brighnessvalprocessed;
      if((brightnessoffset < -500) or (brightnessoffset > 0))//check for realistic values
      { //unrealistic values

        debug("sensor reads : ");
        debugln(analogRead(aSiglight));
        debugln("offset not written : unrealistiv values -> setting offset to 0");

        brightnessoffset = 0;

        errorcode = B00010000;

        switchtrig = true;
        booncestatemachine = false;      
        statemachine = state_fault;


      }
      else
      {//realistic values

        EEPROM.put(EEPROMadrOffset,brightnessoffset);

        debug("sensor reads : ");
        debugln(analogRead(aSiglight));
        debug("Offset written : ");
        debugln(brightnessoffset);

        switchtrig = true;
        booncestatemachine = false;      
        statemachine = state_setup_Brigtness_2;
      }


    }

    break;

/*  ------------------------------------------------------------
                    setup brigness step 2 :  scaling
    ------------------------------------------------------------ */
  case state_setup_Brigtness_2:

    if(booncestatemachine == false)
    {
      booncestatemachine = true;
      displaychar[0] = 's';
      displaychar[1] = 't';
      displaychar[2] = 'p';
      displaychar[3] = '2';
      displaydot[0] = true;
      displaydot[1] = true;
      displaydot[2] = true;
      displaydot[3] = true;
      updatedisplay();
    }

    if (!buttonpressed && switchtrig)//wait for switches to be unpressed
    {
      switchtrig = false;
    }
    else if (switchset.buttonStatus && !switchtrig)// do step 2 of brighness calibration
    {

      brightnessscaling =  ((brighnessvalprocessed + brightnessoffset) / 255 ) + 1;

      if((brightnessscaling > 10) or (brightnessscaling < 1)) //check for realistic values
      {// unrealistic values

        debug("sensor reads : ");
        debugln(analogRead(aSiglight));
        debug("scaling not written : unrealistiv values -> scaling offset to 4");

        brightnessoffset = 0;

        errorcode = B00010000;
        switchtrig = true;
        booncestatemachine = false;      
        statemachine = state_fault;

      } 
      else
      {//realistic values
        EEPROM.put(EEPROMadrScaling,brightnessscaling);
        
        debug("sensor reads : ");
        debugln(analogRead(aSiglight));
        debug("Scaling written : ");
        debugln(brightnessscaling);

        switchtrig = true;
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

    if(booncestatemachine == false)
    {
      whattodisplayBrighness = 0;
      whattodisplayBrighnesslast = 1;
      booncestatemachine = true;

    }
    
    if((whattodisplayBrighness != whattodisplayBrighnesslast) || (whattodisplayBrighness == 0))
    {
      whattodisplayBrighnesslast = whattodisplayBrighness;

      switch (whattodisplayBrighness)
      {
      case 0: //display current light sensor

        displaychar[0] = char(brighnessvalprocessed/1000 %10) + '0';  
        displaychar[1] = char(brighnessvalprocessed/100 %10) + '0';  
        displaychar[2] = char(brighnessvalprocessed/10 %10) + '0';  
        displaychar[3] = char(brighnessvalprocessed/1 %10) + '0';  
        displaydot[0] = true;
        displaydot[1] = false;
        displaydot[2] = false;
        displaydot[3] = false;
        break;   

      case 1://displa offset
        displaychar[0] = char(abs(brightnessoffset) /1000 %10) + '0';  
        displaychar[1] = char(abs(brightnessoffset) /100 %10) + '0';  
        displaychar[2] = char(abs(brightnessoffset) /10 %10) + '0';  
        displaychar[3] = char(abs(brightnessoffset) /1 %10) + '0';  
        displaydot[0] = false;
        displaydot[1] = true;
        displaydot[2] = false;
        displaydot[3] = false;
        break;
      case 2://display scaling
        displaychar[0] = char(brightnessscaling /1000 %10) + '0';  
        displaychar[1] = char(brightnessscaling /100 %10) + '0';  
        displaychar[2] = char(brightnessscaling /10 %10) + '0';  
        displaychar[3] = char(brightnessscaling /1 %10) + '0';  
        displaydot[0] = false;
        displaydot[1] = false;
        displaydot[2] = true;
        displaydot[3] = false;
        break;
      default:
        whattodisplayBrighness = 0;
        whattodisplayBrighnesslast = 1;
        break;
      }
      updatedisplay();
    }

    if(!buttonpressed && switchtrig)//wait for switches to be unpressed
    {
      switchtrig = false;
    }
    else if(switchinc.buttonStatus && !switchtrig)//go to next value
    {
      switchtrig = true;
      whattodisplayBrighness++;
    }
    else if (switchdec.buttonStatus && !switchtrig)//go to last value
    {
      switchtrig = true;
      whattodisplayBrighness--;
      if(whattodisplayBrighness > 2)
      {
        whattodisplayBrighness = 2;
      }
    }
    else if (switchset.buttonStatus && !switchtrig)//go back to the menu
    {
      switchtrig = true;
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
    static uint8_t monthtemp;//local variable for the setting the months because it cant be set directly via incrementaion of seconds
    static uint16_t yeartemp;//local variable for the setting the Years because it cant be set directly via incrementaion of seconds

    if(booncestatemachine == false)
    {
      booncestatemachine = true;
      setupstep = 1;
      setupsteplast = 0;
      rtctimecurrent = Rtc.GetDateTime();//at the start read the present time
      yeartemp = rtctimecurrent.Year();
      monthtemp = rtctimecurrent.Month();
    }


    if((setupstep !=setupsteplast) || ( switchinc.buttonStatus && !switchtrig) || (switchdec.buttonStatus && !switchtrig) )
    {
      setupsteplast = setupstep;

      switch (setupstep)
      {
      case 1://set Year

        if (switchinc.buttonStatus)
        {
          yeartemp++;//increment the temp year
          switchtrig = true;
        }
        else if(switchdec.buttonStatus)
        {
          yeartemp--;//decrement the temp year
          switchtrig = true;
        }

        displaychar[0] = char(yeartemp /1000 %10) + '0';
        displaychar[1] = char(yeartemp /100 %10) + '0';
        displaychar[2] = char(yeartemp /10 %10) + '0';
        displaychar[3] = char(yeartemp /1 %10) + '0';
        displaydot[0] = true;
        displaydot[1] = true;
        displaydot[2] = true;
        displaydot[3] = true;
        break;

      case 2://set Month

        if (switchinc.buttonStatus)
        {
          monthtemp++;//increment the temp month
          switchtrig = true;
          if(monthtemp>12)//month clamping
          {
            monthtemp = 1;
          }
        }
        else if(switchdec.buttonStatus)//month clamping
        {
          monthtemp--;//decrement the temp month
          switchtrig = true;
          if(monthtemp>12)
          {
            monthtemp = 12;
          }
        }

        displaychar[0] = char(monthtemp /10 %10) + '0';
        displaychar[1] = char(monthtemp /1 %10) + '0';
        displaychar[2] = 'm';
        displaychar[3] = 'm';
        displaydot[0] = true;
        displaydot[1] = true;
        displaydot[2] = false;
        displaydot[3] = false;
        break;

      case 3://set Day

        if (switchinc.buttonStatus)
        {
          rtctimecurrent += uint32_t(86400);//increment the time one day
          switchtrig = true;
        }
        else if(switchdec.buttonStatus)
        {
          rtctimecurrent -= 86400;//decrement the time one day
          switchtrig = true;
        }
        
        displaychar[0] = 'd';
        displaychar[1] = 'd';
        displaychar[2] = char( rtctimecurrent.Day() /10 %10) + '0';
        displaychar[3] = char( rtctimecurrent.Day() /1 %10) + '0';
        displaydot[0] = false;
        displaydot[1] = false;
        displaydot[2] = true;
        displaydot[3] = true;
        break;

      case 4://set Hour

        if (switchinc.buttonStatus)
        {
          rtctimecurrent += uint32_t(3600);//increment the time one hour
          switchtrig = true;
        }
        else if(switchdec.buttonStatus)
        {
          rtctimecurrent -= 3600;//decrement the time one hour
          switchtrig = true;
        }

        displaychar[0] = char(rtctimecurrent.Hour() /10 %10) + '0';
        displaychar[1] = char(rtctimecurrent.Hour() /1 %10) + '0';
        displaychar[2] = 'h';
        displaychar[3] = 'h';
        displaydot[0] = true;
        displaydot[1] = true;
        displaydot[2] = false;
        displaydot[3] = false;
        break;

      case 5://set minute

        if (switchinc.buttonStatus)
        {
          rtctimecurrent += uint32_t(60);//increment the time one minute
          switchtrig = true;
        }
        else if(switchdec.buttonStatus)
        {
          rtctimecurrent -= 60;//decrement the time one minute
          switchtrig = true;
        }

        displaychar[0] = 'm';
        displaychar[1] = 'm';
        displaychar[2] = char( rtctimecurrent.Minute() /10 %10) + '0';
        displaychar[3] = char( rtctimecurrent.Minute() /1 %10) + '0';
        displaydot[0] = false;
        displaydot[1] = false;
        displaydot[2] = true;
        displaydot[3] = true;
        break;

      case 6://set Seconds


        if (switchinc.buttonStatus)
        {
          rtctimecurrent += uint32_t(1);//increment the time one Second
          switchtrig = true;
        }
        else if(switchdec.buttonStatus)
        {
          rtctimecurrent -= 1;//decrement the time one Second
          switchtrig = true;
        }

        displaychar[0] = char( rtctimecurrent.Second() /10 %10) + '0';
        displaychar[1] = char( rtctimecurrent.Second() /1 %10) + '0';
        displaychar[2] = 's';
        displaychar[3] = 's';
        displaydot[0] = true;
        displaydot[1] = true;
        displaydot[2] = false;
        displaydot[3] = false;
        break;

      case 7:// write time and go to menu
        { //explicit case here because placeholder crosses initialization
        RtcDateTime placeholder(yeartemp, //the 
          monthtemp,
          rtctimecurrent.Day(),
          rtctimecurrent.Hour(),
          rtctimecurrent.Minute(),
          rtctimecurrent.Second());

          if(placeholder.IsValid() && !(yeartemp <= 2000))//check 
          {
            #if DEBUG == 1
              debug("The time was set to : ");
              printDateTime(rtctimecurrent);
            #endif

            Rtc.SetDateTime(placeholder);// write the time to the RTC if the time is valid
            rtctimecurrent = placeholder;

            booncestatemachine = false;
            statemachine = state_default;            
          }
          else
          {
            debugln("The time was not set due to it beeing not valid");
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

      updatedisplay();
      debug("current Time setting step : ");
      debugln(setupstep);
    }

    if(!buttonpressed && switchtrig)//wait for switches to be unpressed
    {
      switchtrig = false;
    }
    else if(switchinc.buttonStatus && !switchtrig)
    {
      switchtrig = true;
    }
    else if (switchdec.buttonStatus && !switchtrig)
    {
      switchtrig = true;
    }
    else if (switchset.buttonStatus && !switchtrig)
    {
      setupstep = setupstep + 1;
      switchtrig = true;
    }
    
    break;


/*  ------------------------------------------------------------
                    display time
    ------------------------------------------------------------ */
  case state_setup_time_Display_exec:
    static uint8_t whattodisplayTime;
    static uint8_t whattodisplayTimelast;

    if(booncestatemachine == false)
    {
      whattodisplayTime = 0;
      whattodisplayTimelast = 1;
      booncestatemachine = true;
    }
    
    if((whattodisplayTime != whattodisplayTimelast) || whattodisplayTime == 5)
    {
      rtctimecurrent = Rtc.GetDateTime();
      whattodisplayTimelast = whattodisplayTime;

      switch (whattodisplayTime)
      {
      case 0: //display Year
        displaychar[0] = char(rtctimecurrent.Year() /1000 %10) + '0';  
        displaychar[1] = char(rtctimecurrent.Year() /100 %10) + '0';  
        displaychar[2] = char(rtctimecurrent.Year() /10 %10) + '0';  
        displaychar[3] = char(rtctimecurrent.Year() /1 %10) + '0';  
        displaydot[0] = true;
        displaydot[1] = false;
        displaydot[2] = false;
        displaydot[3] = false;
        break;   
      case 1://display month
        displaychar[0] = 'm';  
        displaychar[1] = 'm';  
        displaychar[2] = char(rtctimecurrent.Month() /10 %10) + '0';  
        displaychar[3] = char(rtctimecurrent.Month() /1 %10) + '0';  
        displaydot[0] = false;
        displaydot[1] = true;
        displaydot[2] = false;
        displaydot[3] = false;
        break;
      case 2://display day
        displaychar[0] = 'd';  
        displaychar[1] = 'd';  
        displaychar[2] = char(rtctimecurrent.Day() /10 %10) + '0';  
        displaychar[3] = char(rtctimecurrent.Day() /1 %10) + '0';  
        displaydot[0] = false;
        displaydot[1] = false;
        displaydot[2] = true;
        displaydot[3] = false;
        break;
      case 3://display hour
        displaychar[0] = 'h';  
        displaychar[1] = 'h';  
        displaychar[2] = char(rtctimecurrent.Hour() /10 %10) + '0';  
        displaychar[3] = char(rtctimecurrent.Hour() /1 %10) + '0';  
        displaydot[0] = false;
        displaydot[1] = false;
        displaydot[2] = false;
        displaydot[3] = true;
        break;
      case 4://display minutes
        displaychar[0] = 'm';  
        displaychar[1] = 'm';  
        displaychar[2] = char(rtctimecurrent.Minute() /10 %10) + '0';  
        displaychar[3] = char(rtctimecurrent.Minute() /1 %10) + '0';  
        displaydot[0] = true;
        displaydot[1] = true;
        displaydot[2] = false;
        displaydot[3] = false;
        break;
      case 5://display seconds
        displaychar[0] = 's';  
        displaychar[1] = 's';  
        displaychar[2] = char(rtctimecurrent.Second() /10 %10) + '0';  
        displaychar[3] = char(rtctimecurrent.Second() /1 %10) + '0';  
        displaydot[0] = true;
        displaydot[1] = true;
        displaydot[2] = true;
        displaydot[3] = false;
        break;
      default:
        whattodisplayTime = 0;
        whattodisplayTimelast = 1;
        break;
      }
      updatedisplay();
    }

    if(!buttonpressed && switchtrig)//wait for switches to be unpressed
    {
      switchtrig = false;
    }
    else if(switchinc.buttonStatus && !switchtrig)//go to next value
    {
      switchtrig = true;
      whattodisplayTime++;
    }
    else if (switchdec.buttonStatus && !switchtrig)//go to last value
    {
      switchtrig = true;
      whattodisplayTime--;
      if(whattodisplayTime > 5)
      {
        whattodisplayTime = 5;
      }
    }
    else if (switchset.buttonStatus && !switchtrig)//go back to the menu
    {
      statemachine = state_setup_time_Display_menu;
      switchtrig = true;
      booncestatemachine = false;
    }
    
    break;

/*  ------------------------------------------------------------------------------------------------------------------------
                    Reset time : ask again
    ------------------------------------------------------------------------------------------------------------------------ */
  case state_setup_resetcounter_reask:

     if(booncestatemachine == false)
    {
      booncestatemachine = true;
      displaychar[0] = 's';
      displaychar[1] = 'u';
      displaychar[2] = 'r';
      displaychar[3] = 'e';
      displaydot[0] = true;
      displaydot[1] = true;
      displaydot[2] = true;
      displaydot[3] = true;
      updatedisplay();
    }

    if(!buttonpressed && switchtrig)//wait for switches to be unpressed
    {
      switchtrig = false;
    }

    else if (switchset.buttonStatus && !switchtrig)//navigate to the parent menu
    {
      switchtrig = true;
      booncestatemachine = false;
      statemachine = state_setup;
    }
    else if (switchdec.buttonStatus && switchinc.buttonStatus && !switchtrig)//if both inc and dec are pressed at the same time, a timer of 5 seconds is startet
    {

      if(millis() - timerton >= 5000) // the two switches have to be held 5 seconds
      {

        rtctimecurrent = Rtc.GetDateTime();//sync both times
        rtctimeVfree = Rtc.GetDateTime();
        EEPROM.put(EEPOMadrStarttime,rtctimeVfree);//write time to EEPROM

        statemachine = state_default;//go back to menu after reset
        switchtrig = true;
        booncestatemachine = false;
        debugln("Time since last incident was reset to current time");
      }    
      
    }
    else //reset timer
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
    byte errorworkingvalue = errorcode;
    static byte lasterror;
    static bool alldisplayed;

    if(booncestatemachine == false)
    {
      debug("Ther is a error : ");
      debugln(errorcode);
      booncestatemachine = true;
      timerton = millis();
      lasterror = 0;
      alldisplayed = false;
      displaychar[0] = 'e';  
      displaychar[1] = 'r';  
      displaychar[2] = 'r';  
      displaychar[3] = ' ';  
      displaydot[0] = false;
      displaydot[1] = false;
      displaydot[2] = false;
      displaydot[3] = true;
      updatedisplay();
  
    }


    if((millis() - timerton >= 5000) && !alldisplayed)
    {
      debug("What is going on with all displayed : ");
      debugln(alldisplayed);
      timerton = millis();
      for (byte i = 1; i <= 6; i++)//run through all the errors 
      {
        debug("for states : ");
        debugln(i);
        debugln(errorworkingvalue);

        if (((errorworkingvalue & B00000001) == B00000001) && (i > lasterror))// check if the error bit was set or the error was already displayed
        {
          lasterror = i;
          displaychar[3] = char(lasterror) + '0';  
          break;
        }

        if(i == 6) //if all errors have been displayed continue or wait for a reset
        {
          alldisplayed = true;
        }
        errorworkingvalue = errorworkingvalue>>1;
      }
      updatedisplay();
    }

    if(alldisplayed && !erroroccurednorecovery)//When every error has been displayed and the error is recoverable, go to normal operation. If not a reset is needed
    {
      debugln("All errors have been displayed");
      errorcode = 0;
      statemachine = state_noinit;
    }
    else if(alldisplayed && erroroccurednorecovery)
    {
      displaychar[0] = 'e';  
      displaychar[1] = 'r';  
      displaychar[2] = 'r';  
      displaychar[3] = 'f';
      updatedisplay();  
    }
  }
    break;

/*  ------------------------------------------------------------
                    default (unknown state)
    ------------------------------------------------------------ */
  default:

    debug("unknown state : ");
    debugln(statemachine);
    booncestatemachine = false;
    switchtrig = false;
    statemachine = state_noinit;
    break;
  }
}

//Updating the Display with new data
void updatedisplay()
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

  //set shift and data pins to known values  
  digitalWrite(dpClk,false);
  digitalWrite(dpData,false);

  debugln("Display update ------------");
  debug("string written to registers : ");
  debugln(displaychar);
  

  for (int i = 0; i <= 3; i++)
  {
  
   switch (displaychar[3-i])
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
      dataforshift = B11110110;
      break;
    }

    if(displaydot[3-i])
    {
      dataforshift = dataforshift | B00000001;
    }

    shiftOut(dpData,dpClk,MSBFIRST,dataforshift);

    debug("Byte ");
    debug(3-i);
    debug(" : ");
    #if DEBUG == 1
      Serial.println(dataforshift,BIN);
    #endif
    
  }

  //display the writen characters
  digitalWrite(dpRclkU4,true);
  digitalWrite(dpRclkU5,true);
  digitalWrite(dpRclkU6,true);
  digitalWrite(dpRclkU7,true);
  delay(1);
  digitalWrite(dpRclkU4,false);
  digitalWrite(dpRclkU5,false);
  digitalWrite(dpRclkU6,false);
  digitalWrite(dpRclkU7,false);
  delay(1);
  digitalWrite(dpClk,false);
  digitalWrite(dpData,false);

  debugln("display updated ------------");


}

//adjust the brightness to ambient light setting
void setbrightness()
{
  //This part of the function reads the sensor and smooths the value

  static int sensvalues[10];
  static byte sensindex;

  sensvalues[sensindex] = analogRead(aSiglight);
  sensindex++;

  if(sensindex >= 10)
  {
    sensindex = 0;
  }

  for (byte i = 0; i < 10; i++)
  {
    brighnessvalprocessed = brighnessvalprocessed + sensvalues[i];
  }

  brighnessvalprocessed = brighnessvalprocessed / 10;

  //this function calculates the brighness value

  int brightnesstowrite;
  brightnesstowrite =  brighnessvalprocessed;
  brightnesstowrite = brightnesstowrite + brightnessoffset;
  brightnesstowrite = brightnesstowrite / brightnessscaling;

  if(brightnesstowrite> 255)
  {
    brightnesstowrite = 255;
  }

  #if allowdisplayoff == 0
  else if(brightnesstowrite<1)
  {
    brightnesstowrite = 1;
  }
  #endif

  analogWrite(dplight,brightnesstowrite);
  
  #if DEBUG == 1 //only print the Brighnessvalue every 5 seconds to not spam the Serial port
    static long timeforbrighness;
    if(millis() - timeforbrighness >= 5000 ) 
    {
      timeforbrighness = millis();
      debug("brightness set to ");
      debugln(brightnesstowrite);
    }

  #endif  
  
}

//debounce switches
void switchhandler()
{
  switchset.scan();
  switchinc.scan();
  switchdec.scan();

  buttonpressed = switchset.buttonStatus || switchinc.buttonStatus || switchdec.buttonStatus;
}

#if DEBUG == 1 //this function is only used for debuging
  void printDateTime(const RtcDateTime& dt)
  {
      char datestring[20];
      debug("the time is : ");
      snprintf_P(datestring, 
              countof(datestring),
              PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
              dt.Month(),
              dt.Day(),
              dt.Year(),
              dt.Hour(),
              dt.Minute(),
              dt.Second() );
      debugln(datestring);
  }
#endif

void calcdisplaydefault(bool reload) //calculate the default display
{

  static unsigned long secondsofday; //seconds until it is midnight maximum is 86400
  static unsigned long lastmilisread; //timer to reduce RTC reads. the RTC gets read only
  static uint16_t dayspassed; //the count of days passed 
  static uint16_t dayspassedlast; //count of days comparrison

  if(reload  || (86370 <= (secondsofday + (( millis() - lastmilisread) / 1000 ))) || (30 >= (secondsofday + (( millis() - lastmilisread) / 1000 ))) )//when the time until midnight is within 30 seconds the RTC time gets read (I try to not read the RTC too much)
  {

    rtctimecurrent = Rtc.GetDateTime();
    dayspassed = rtctimecurrent.TotalDays() - rtctimeVfree.TotalDays();
    secondsofday = (long(rtctimecurrent.Hour()) * 3600) + (long(rtctimecurrent.Minute()) * 60 ) + long(rtctimecurrent.Second());
    lastmilisread = millis();

    debug("seconds of the day : ");
    debugln(secondsofday);
    debug("days passed rtc : ");
    debugln(rtctimecurrent.TotalDays());
    debug("days passed Vfree : ");
    debugln(rtctimeVfree.TotalDays());

  }

  if((dayspassed != dayspassedlast) || reload)
  {

    reload = false;
    bool numberhere = false;
    debugln("displaying days passed");
    dayspassedlast = dayspassed;

    if(((dayspassed / 1000  % 10 ) == 0) && !numberhere)
    {
      displaychar[0] = ' ';
    }
    else
    {
      numberhere = true;
      displaychar[0] = char(dayspassed / 1000  % 10) + '0';
    }

    if(((dayspassed / 100  % 10 ) == 0) && !numberhere)
    {
      displaychar[1] = ' ';
    }
    else
    {
      numberhere = true;
      displaychar[1] = char(dayspassed / 100  % 10) + '0';
    }

    if(((dayspassed / 10  % 10 ) == 0) && !numberhere)
    {
      displaychar[2] = ' ';
    }
    else
    {
      numberhere = true;
      displaychar[2] = char(dayspassed / 10  % 10) + '0';
    }

    displaychar[3] = char(dayspassed / 1  % 10) + '0';

    updatedisplay();
  } 
}

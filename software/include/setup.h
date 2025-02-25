#ifndef setup_h
#define setup_h

#include <arduino.h>
#include <avr/pgmspace.h>

// setup for debug

#define DEBUG 0 // In normal use this should be a 0 only set this to a 1 if you want to debug the code

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

// option
#define allowdisplayoff 0 // if you set this to 1 the display can be fully off when the light level is sufficiently low

// Defining I/O's
// Display
constexpr auto dpClear = 8;
constexpr auto dpData = 9;
constexpr auto dpOe = 6;
constexpr auto dpClk = 7;
constexpr auto dpRclkU4 = 5;
constexpr auto dpRclkU5 = 4;
constexpr auto dpRclkU6 = 3;
constexpr auto dpRclkU7 = 2;
constexpr auto dplight = 10;
// switches
constexpr auto sSet = A3;
constexpr auto sInc = A2;
constexpr auto sDec = A1;
// Photo transistor
constexpr auto aSiglight = A0;

// I2C Adress of RTC
constexpr auto rtcadress = B1101000;

// EEpromadresses
constexpr auto EEPROMadrOffset = 1;    // offsetadress for brighness setting
constexpr auto EEPROMadrScaling = 21;  // scaling adress for scaling adress
constexpr auto EEPOMadrStarttime = 41; // starting day adress

// timerintervall for the display brightness adjustment
constexpr auto calltime = 50L;

// enumerator of the different states that the menu can be
enum systemstate
{
    SysState_noInit,                      // The state machine has not been set and sets some variables and steps to the next step
    SysState_fault,                       // A error has occured and the user has to be informed
    SysState_idleUnlocked,                // the menu is in its default state of displaying passed days with unlocked switches
    SysState_idleLocked,                  // the menu is in its default state of displaying passed days with locked switches
    SysState_menu_Main,                   // the menu is in the setup mode and the setting can be selected that want to be changed
    SysState_menu_calibration,            // setupmenu for setting the light menu
    SysState_menu_displayCalibrationData, // displaying the values asociated with the Brighness setting
    SysState_menu_displayTime,            // dislay the time menu option
    SysState_menu_displayVTime,           // Menu option for displaying the velociraptor incident time
    SysState_menu_resetCounter,           // setupmenu for resetting the counter
    SysState_menu_timeSet,                // setupmenu for setting the time
    SysState_menu_VtimeSet,               // setupmenu for setting the velociraptor incident time (only during debugging)
    SysState_display_calibrationData,     // displaying the values asociated with the Brighness setting
    SysState_display_time,                // dislay the time
    SysState_display_VTime,               // dislay the VelociraptorTime
    SysState_setup_calibrationStp1,       // step 1 in brightness calibration (determine zero offset)
    SysState_setup_calibrationStp2,       // step 2 in brightness calibration (determin scaling factor)
    SysState_setup_resetCounterConfirm,   // Ask for confirmation to reset counter
    SysState_setup_time,                  // setting the time
    SysState_setup_VTime,                 // setting the velociraptor incident time (only during debugging)
    SysState_NumOfTypes                   // used to check if the number of states is correct
};

enum errorstate // binary error codes, that way all can be active at the same time
{
    error_RTCtime = B00000001,     // RTC time not valid
    error_RTCfatal = B00000010,    // RTC fatal error
    error_invalidtime = B00000100, // invalid time for the last incident
    error_brighness = B00001000,   // Brighness has invalid calibration values
    error_unrealistic = B00010000, // calibration values are unrealistic
    error_Nullpointer = B00100000, // Nullpointer detected where there should be none
    error_NumOfTypes               // used to check for max number of errors
};

// nullptr signifies no default setting needed
const char *defaultDisplaysStr[] = // PROGMEM TOTEST
    {
        "ini ",  // SysState_noInit
        "err ",  // SysState_fault
        nullptr, // SysState_idleUnlocked
        nullptr, // SysState_idleLocked
        "set ",  // SysState_menu_Main
        "ambl",  // SysState_menu_calibration
        "amb?",  // SysState_menu_displayCalibrationData
        "tme?",  // SysState_menu_displayTime
        "vtim",  // SysState_menu_displayVTime
        "res?",  // SysState_menu_resetCounter
        "time",  // SysState_menu_timeSet
        "tmev",  // SysState_menu_VTimeSet
        nullptr, // SysState_display_calibrationData
        nullptr, // SysState_display_time
        nullptr, // SysState_display_VTime
        "stp1",  // SysState_setup_calibrationStp1
        "stp2",  // SysState_setup_calibrationStp2
        "sure",  // SysState_setup_resetCounterConfirm
        nullptr, // SysState_setup_time
        nullptr  // SysState_setup_VTime
};

// entrys where defaultDisplaysStr a null is are only padding because data is not used
constexpr byte defaultDisplaysByte[] =
    {
        B0000, // SysState_noInit
        B1111, // SysState_fault
        B0000, // SysState_idleUnlocked
        B0000, // SysState_idleLocked
        B1111, // SysState_menu_Main
        B0001, // SysState_menu_calibration
        B0010, // SysState_menu_displayCalibrationData
        B0100, // SysState_menu_displayTime
        B1000, // SysState_menu_displayVTime
        B1111, // SysState_menu_resetCounter
        B1100, // SysState_menu_timeSet
        B1010, // SysState_menu_VtimeSet
        B0000, // SysState_display_calibrationData
        B0000, // SysState_display_time
        B0000, // SysState_display_VTime
        B0011, // SysState_setup_calibrationStp1
        B1111, // SysState_setup_calibrationStp2
        B1111, // SysState_setup_resetCounterConfirm
        B0000, // SysState_setup_time
        B0000  // SysState_setup_VTime
};

#endif // includeguard
#ifndef setup_h
#define setup_h

#include <arduino.h>

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

// timerintervall for

constexpr auto calltime = 50L;

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

enum errorstate // binary error codes, that way all can be active at the same time
{
    error_RTCtime = B00000001,     // RTC time not valid
    error_RTCfatal = B00000010,    // RTC fatal error
    error_invalidtime = B00000100, // invalid time for the last incident
    error_brighness = B00001000,   // Brighness has invalid calibration values
    error_unrealistic = B00010000  // calibration values are unrealistic
};

const char *defaultDisplaysStr[] = {"ini ",  // state_noinit
                                    "err ",  // state_fault
                                    nullptr, // state_default
                                    nullptr, // state_locked
                                    "set ",  // state_setup
                                    "ambl",  // state_setup_Brigtness_menu
                                    "amb?",  // state_setup_Brigtness_Displayvalue_menu
                                    nullptr, // state_setup_Brigtness_Displayvalue_exec
                                    "stp1",  // state_setup_Brigtness_1
                                    "stp2",  // state_setup_Brigtness_2
                                    "time",  // state_setup_time_menu
                                    nullptr, // state_setup_time_set
                                    "tim?",  // state_setup_time_Display_menu
                                    nullptr, // state_setup_time_Display_exec
                                    "res?",  // state_setup_resetcounter_menu
                                    "sure"}; // state_setup_resetcounter_reask

constexpr byte defaultDisplaysByte[] = {B0000,  // state_noinit
                                        B0001,  // state_fault
                                        B0000,  // state_default
                                        B0000,  // state_locked
                                        B1111,  // state_setup
                                        B1000,  // state_setup_Brigtness_menu
                                        B0100,  // state_setup_Brigtness_Displayvalue_menu
                                        B0000,  // state_setup_Brigtness_Displayvalue_exec
                                        B1100,  // state_setup_Brigtness_1
                                        B1111,  // state_setup_Brigtness_2
                                        B0010,  // state_setup_time_menu
                                        B0000,  // state_setup_time_set
                                        B0001,  // state_setup_time_Display_menu
                                        B0000,  // state_setup_time_Display_exec
                                        B1001,  // state_setup_resetcounter_menu
                                        B1111}; // state_setup_resetcounter_reask

#endif // includeguard
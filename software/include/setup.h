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

#endif // includeguard
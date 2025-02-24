#pragma once
#include <Arduino.h>

/**
 * @brief This class implements simple timer that work off of the internal millis
 * that means it may not be that accurate but it should be enough
 *
 *
 **/

enum STimer_State
{
    STimer_State_TON,
    STimer_State_TOFF,
    STimer_State_TFF
};

class STimer
{
public:
    STimer(unsigned char _timerFunc = 0, unsigned long _timeWait = 1000); // constructor with initlist

    bool out = false; // Output for Timerstatus, dependent on the function of the Timer

    /**
     * @brief when this func is called, the timer runs its function
     *
     **/
    void call();

    /**
     * @brief sets the timer mode (default is 0), it also resets the timer
     * 0 : TON  -> After reset, the output is set to off. The timer then waits the timeWait, then turns the output on (it does nothing until reset or func change).
     * 1 : TOFF -> After reset, the output is set to on. The timer then waits the timeWait, then turns the output off (it does nothing until reset or func change).
     * 2 : TFF  -> After reset, the output is set to off. The timer then waits the timeWait, then toggles the output.
     *
     **/
    // void setState(unsigned char state = 0); // currently not needed

    /**
     * @brief resets the timer (depending on the output it also sets the output to different values)
     *
     *
     **/
    void resetTimer();

private:
    unsigned char timerFunc; // the current function of the timer

    unsigned long timerWait; // the time that is used for the different functions
    unsigned long milliTime; // internal time
    void (STimer::*timerFuncPtr)(void) = nullptr;
    void timerTon();
    void timerToff();
    void timerTFF();
};
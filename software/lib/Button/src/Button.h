#ifndef BUTTON_h
#define BUTTON_h

#include <Arduino.h>

constexpr long debouncetime = 50; // debouunce time in ms

/**
 * @brief This class handles a physical button. Buttonstatus is debounced and will always have clean transitions
 *
 *
 **/
class Button
{
public:
    Button(unsigned int _hardwareAdress); // constructor with initlist
    bool buttonStatus;                    // true if the button is pressed

    /**
     * @brief function to be called if a button event is detected.
     *
     **/
    void scan();

    /**
     * @brief returns a buttonpush only once after a button has been pushed.
     * Resets after the button has been depressed
     */
    bool trigger();

private:
    unsigned int hardwareAdress; // hardware adress to the button
    unsigned long milliTime;
    unsigned long milliTimePrev;
    bool wasTriggered;
};

#endif
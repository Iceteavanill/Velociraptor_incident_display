#include "Button.h"

Button::Button(unsigned int _hardwareAdress) : hardwareAdress{_hardwareAdress}
{
    // not used
}

void Button::scan()
{

    bool switchRead = digitalRead(hardwareAdress);

    if (switchRead != buttonStatus) // check for change in Button state
    {
        if (((millis() - milliTime) >= debouncetime)) // wait for bounceTime to pass
        {
            buttonStatus = switchRead; // update switchstate
            milliTime = millis();
        }
    }
    else
    {
        milliTime = millis(); // update milliTime to current time
    }

    wasTriggered &= buttonStatus; // reset wasTriggered once the button is depressed
}

bool Button::trigger()
{

    if (wasTriggered || !buttonStatus)
    {
        return false;
    }
    else
    {
        wasTriggered = true;
        return buttonStatus;
    }
}

#pragma once
/**
 * @brief Simple statemachien handler. Implements some usefull functions to handle states
 *
 *
 **/
class StateController
{
public:
  /**
   * @brief Construct a new State Controller object
   */
  StateController(unsigned int _activeStep, void (*fptr)(unsigned int) = nullptr);
  /**
   * @brief Always jump to the next State
   *
   **/
  void nextStep(unsigned int inStp);
  /**
   * @brief jump to the next State if the second argument is true
   *
   **/
  void nextStepConditional(unsigned int inStp, bool inCond);
  unsigned int activeStep = 0;
  unsigned int lastStep = 0;
  /**
   * @brief returns true only one time
   *
   **/
  bool doOnce(); // is only active for one cycle

  /**
   * @brief resets the stateController as if it was new
   */
  void reset(unsigned int _activeStep = 0);

private:
  void (*actionFunc)(unsigned int) = nullptr;
  unsigned int lastStepForDoOnce = 0;
};
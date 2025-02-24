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
  StateController(unsigned int _activeStep = 0, void (*fptr)(unsigned int) = nullptr);

  /**
   * @brief Always jump to the next State
   *
   **/
  void nextStep(unsigned int inStp);

  /**
   * @brief jump to the next State if ture
   */
  void incrementStep(bool inCond);

  /**
   * @brief jump to the previous State if true
   **/
  void decrementStep(bool inCond);

  /**
   * @brief jump to the next State if the second argument is true
   *
   **/
  void nextStepConditional(unsigned int inStp, bool inCond);

  /**
   * @brief returns true only one time
   *
   **/
  bool doOnce(); // is only active for one cycle

  /**
   * @brief sets step range
   */
  void setStepRange(unsigned int _stepRange);

  /**
   * @brief resets the stateController as if it was new
   */
  void reset(unsigned int _activeStep = 0, unsigned int _steprange = 0);

  unsigned int activeStep = 0; // contains the current state
  unsigned int lastStep = 0;   // contains the last state

private:
  void (*actionFunc)(unsigned int) = nullptr;
  unsigned int lastStepForDoOnce = 0;
  unsigned int stepRange = 0; // limit step range(count of). If 0 no limit. only active for inc or decrement
};
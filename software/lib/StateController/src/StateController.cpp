#include "StateController.h"

StateController::StateController(unsigned int _activeStep, void (*fptr)(unsigned int) = nullptr) : activeStep(_activeStep), actionFunc(fptr)
{
  // not used
}

void StateController::nextStep(unsigned int inStp)
{
  lastStep = activeStep;
  activeStep = inStp;
  if (actionFunc != nullptr)
  {
    actionFunc(activeStep);
  }
}

void StateController::nextStepConditional(unsigned int inStp, bool inCond)
{
  if (inCond)
  {
    lastStep = activeStep;
    activeStep = inStp;
    if (actionFunc != nullptr)
    {
      actionFunc(activeStep);
    }
  }
}

bool StateController::doOnce()
{
  if (lastStepForDoOnce != activeStep)
  {
    lastStepForDoOnce = activeStep;
    return true;
  }
  else
  {
    return false;
  }
}

void StateController::reset(unsigned int _activeStep = 0)
{
  activeStep = _activeStep;
  lastStep = 0;
  lastStepForDoOnce = 0;
}
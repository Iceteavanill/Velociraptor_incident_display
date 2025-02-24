#include "StateController.h"

StateController::StateController(unsigned int _activeStep, void (*fptr)(unsigned int)) : activeStep(_activeStep), actionFunc(fptr)
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

void StateController::reset(unsigned int _activeStep, unsigned int _steprange)
{
  activeStep = _activeStep;
  lastStep = activeStep + 1;
  lastStepForDoOnce = activeStep + 1;
  stepRange = _steprange;
}

void StateController::incrementStep(bool inCond)
{
  if (inCond)
  {
    lastStep = activeStep;
    if (activeStep == stepRange - 1)
    {
      activeStep = 0;
    }
    else
    {
      activeStep++;
    }
    if (actionFunc != nullptr)
    {
      actionFunc(activeStep);
    }
  }
}

void StateController::decrementStep(bool inCond)
{
  if (inCond)
  {
    lastStep = activeStep;
    if (activeStep == 0)
    {
      activeStep = stepRange - 1;
    }
    else
    {
      activeStep--;
    }
    if (actionFunc != nullptr)
    {
      actionFunc(activeStep);
    }
  }
}

void StateController::setStepRange(unsigned int _stepRange)
{
  stepRange = _stepRange;
}

#ifndef CONTROLLER_INCLUDE_GUARD
#define CONTROLLER_INCLUDE_GUARD
#include <Arduino.h>
#include "Button.h"
#include <AccelStepper.h>
#include <elapsedMillis.h>
#include "ControllerStateMachine.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Motor Connections (constant voltage bipolar H-bridge motor driver)
const int AIn1 = 40;
const int AIn2 = 42;
const int BIn1 = 44;
const int BIn2 = 46;

const int modePin = 52;
const int hitPin = 24;
const int LeftLimitSwitchPin = 50;
const int RightLimitSwitchPin = 48;

const unsigned int speedAdjustmentPin = A0;
const unsigned int accelAdjustmentPin = A4;

enum states 
{
  STOP,
  INITIALIZING,
  LINEAR,
  LINEAR_COMPETITION,
  RANDOM,
};
struct stringStates {
   static const char* enumtext[];
};

class controller 
{
  public:
  controller();

  void init();

  void changeCB(int state);

  void handleLimitSwitches();

  void handleButtons();

  void handleKnobs();

  void run();

  private:
  // utilities
  void updateTargetTowardsCenter(unsigned long int steps);
  void updateTargetToOtherSide();
  bool entryDelay();
  void randomPosition();
  void randomSpeed();
  void randomAccl();
  
  // Default implementations
  bool doNothingEntry();
  bool doNothingRun();
  bool doNothingExit();
  bool genericExit();

  // Stop state specific functions
  bool stopEntry();

  // Init state specific functions
  bool initEntry();
  bool initRun();

  // Linear state specific functions
  bool linearEntry();
  bool linearRun();

  // Linear competetion specific functions
  bool linearCompEntry();
  bool linearCompRun();

  // random specific functions
  bool randomEntry();
  bool randomRun();

  public:
  AccelStepper mStepper;
  StateMachine mStateMachine;
  const float mMaxSpeedLimit = 800;  // set this to the maximum speed you want to use.
  const float mMaxAcceleration = 3200;
  const long int mMaxDistance = 2500; // 90mm diameter wheel with 200 pulses per revolution -> 1,4137mm/pulse. Max supported distance 3,5m -> approx 2500 pulses.
  const long int mMarginFromEnd = 50;
  float mSpeed = 500;
  float mAcceleration = 450;  // set this to the speed we are currently moving at (if acceleration phase is over)
  Button mModeButton;
  Button mHitButton;
  Button mPositiveLimitSwitch;
  Button mNegativeLimitSwitch;

  Adafruit_SSD1306 display;

  long int mTargetPos = mMaxDistance;

  elapsedMillis mTimer;
  uint8_t mLedState = LOW;

  long int mPositiveLimit = mMaxDistance;
  long int mNegativeLimit = -mMaxDistance;
  bool mHitRegistered = false;

  bool mFirstEntryDelay = true;

  bool mInitFoundFirst = false;

  bool mLinearCompInit = false;

  enum LinearComp {RUN, WON, CELEBRATE};
  LinearComp mLinearCompState = LinearComp::RUN;
  unsigned int mShakes = 0;
  long int mDelta = 50;

  elapsedMillis mEntryTimer;

};


#endif /* CONTROLLER_INCLUDE_GUARD */
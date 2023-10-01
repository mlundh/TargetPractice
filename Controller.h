
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
const int dirPin = 11;
const int stepPin = 12;
const int enablePin = 13;

const int MS1 = 40;
const int MS2 = 38;

const int modePin = 52;
const int stopPin = 48;
const int hitPin = 10;
const int negativeLimitSwitchPin = 26;
const int positiveLimitSwitchPin = 30;

const unsigned int speedAdjustmentPin = A3;
const unsigned int accelAdjustmentPin = A4;
const unsigned int randSeedPin = A0;


const int stepMultiplier = 5*4;

enum states 
{
  STOP,
  INITIALIZING,
  LINEAR,
  LINEAR_REACT,
  LINEAR_COMPETITION,
  RANDOM,
  MOOSE,
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
  bool randomDelay(); 

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
  bool initExit();
  
  // Linear state specific functions
  bool linearEntry();
  bool linearRun();

  // Linear state specific functions
  bool linearReactEntry();
  bool linearReactRun();

  // Linear competetion specific functions
  bool linearCompEntry();
  bool linearCompRun();

  // random specific functions
  bool randomEntry();
  bool randomRun();

  // random specific functions
  bool mooseEntry();
  bool mooseRun();

  public:
  AccelStepper mStepper;
  StateMachine mStateMachine;
  const float mMaxSpeedLimit = 800*stepMultiplier;  // set this to the maximum speed you want to use.
  const float mMaxAcceleration = 600*stepMultiplier;
  const float mStopFastAcc = mMaxAcceleration * 2;
  const long int mMaxDistance = 5600*stepMultiplier; // 90mm diameter wheel with 200 pulses per revolution -> 1,4137mm/pulse. Max supported distance 8m -> approx 5600 pulses. 5:1 gearbox, and quarterstep.
  const long int mMarginFromEnd = 5*stepMultiplier;
  const long int mMaxDelay = 2000;
  const long int mInitDelay = 1000;

  float mSpeed = 500*stepMultiplier;
  float mAcceleration = 450*stepMultiplier;  // set this to the speed we are currently moving at (if acceleration phase is over)
  Button mModeButton;
  Button mHitButton;
  Button mStopButton;
  Button mPositiveLimitSwitch;
  Button mNegativeLimitSwitch;


  long int mTargetPos = mMaxDistance;
  uint8_t mLedState = LOW;
  long int mPositiveLimit = mMaxDistance;
  long int mNegativeLimit = -mMaxDistance;
  long unsigned int mTrackLength = 0;
  bool mHitRegistered = false;
  bool mFirstEntryDelay = true;
  bool mInitFoundFirst = false;
  bool mLinearCompInit = false;
  bool mMooseInit = false;
  unsigned int mShakes = 0;
  long int mDelta = 150;
  bool mFirstRandDelay = true;
  long int mRandDelay = 0;

  enum LinearComp {RUN, WON, CELEBRATE};
  enum MooseState {START, DELAY, GO};

  MooseState mMooseState = MooseState::START;
  LinearComp mLinearCompState = LinearComp::RUN;

  elapsedMillis mEntryTimer;
  elapsedMillis mRandTimer;
  elapsedMillis mMooseTimer;
  Adafruit_SSD1306 display;

};


#endif /* CONTROLLER_INCLUDE_GUARD */
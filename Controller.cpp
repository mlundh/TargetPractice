#include "variant.h"
#include "Controller.h"
#include <Wire.h>

volatile uint8_t modeButtonPressEvent = LOW;
volatile uint8_t hitButtonPressEvent = LOW;
volatile uint8_t LeftLimitSwitchEvent = LOW;
volatile uint8_t RightLimitSwitchEvent = LOW;


void ISR_modeButton() 
{
  modeButtonPressEvent = HIGH;
}
void ISR_hitButton() 
{
  hitButtonPressEvent = HIGH;
}

void ISR_LeftLimitSwitch() 
{
  LeftLimitSwitchEvent = HIGH;
}

void ISR_RightLimitSwitch() 
{
  RightLimitSwitchEvent = HIGH;
}

controller::controller()
  : mStepper(AccelStepper::FULL4WIRE, AIn1, AIn2, BIn1, BIn2),
    mModeButton(modePin, ISR_modeButton, &modeButtonPressEvent, "mode"),
    mHitButton(hitPin, ISR_hitButton, &hitButtonPressEvent, "hit"),
    mPositiveLimitSwitch(LeftLimitSwitchPin, ISR_LeftLimitSwitch, &LeftLimitSwitchEvent, "LeftLimit"),
    mNegativeLimitSwitch(RightLimitSwitchPin, ISR_RightLimitSwitch, &RightLimitSwitchEvent, "RightLimit"),
    mStateMachine()
{
  mStepper.setEnablePin(38);
  mStepper.setMaxSpeed(mSpeed);
  mStepper.setAcceleration(mAcceleration);
  
  mStateMachine.registerState("STOP", std::bind(&controller::stopEntry, this), std::bind(&controller::doNothingRun, this), std::bind(&controller::doNothingExit, this));
  mStateMachine.registerState("INIT", std::bind(&controller::initEntry, this), std::bind(&controller::initRun, this), std::bind(&controller::doNothingExit, this));
  mStateMachine.registerState("LINEAR", std::bind(&controller::linearEntry, this), std::bind(&controller::linearRun, this), std::bind(&controller::doNothingExit, this));
  mStateMachine.registerState("LINEAR_COMPETITION", std::bind(&controller::linearCompEntry, this), std::bind(&controller::linearCompRun, this), std::bind(&controller::doNothingExit, this));
  mStateMachine.registerState("RANDOM", std::bind(&controller::randomEntry, this), std::bind(&controller::randomRun, this), std::bind(&controller::doNothingExit, this));
};


void controller::handleLimitSwitches() 
{
  bool positivePressed = mPositiveLimitSwitch.pressed();
  bool negativePressed = mNegativeLimitSwitch.pressed();

  if ( positivePressed || negativePressed ) 
  {

    mStepper.setAcceleration(10000);  // Stopp quickly, set acceleration to a high level!
    mStepper.stop();
    while (mStepper.run())
    {

    }                                         // stop as fast as possible. Let accelStepper lib stop the motor.
    mStepper.setAcceleration(mAcceleration);  // reset the acceleration so that function can resume.
    //mStepper.disableOutputs();               // then disable outputs.
    mStepper.moveTo(mStepper.currentPosition()); // move to current position, this means stop here and wait for new command.
    
    if(positivePressed)
    {
      mPositiveLimit = mStepper.currentPosition() - mMarginFromEnd;
      Serial.print("positive limit reached: ");
      Serial.println(mPositiveLimit);   

    }
    else
    {
      mNegativeLimit = mStepper.currentPosition() + mMarginFromEnd;
      Serial.print("negative limit reached: ");   
      Serial.println(mNegativeLimit);   
    }

  }
}
void controller::handleButtons() 
{
  if (mModeButton.pressed()) 
  {
    mStateMachine.changeState();
  }
  if (mHitButton.pressed()) 
  {
    Serial.println("HIT!");
    mHitRegistered = true;
  }
}

void controller::handleKnobs() {
  if (mTimer >= 1000) {
    mTimer = 0;
    int speedSensorReading = analogRead(speedAdjustmentPin);
    // map it to a range from 0 to 100:
    int tmpSpeed = map(speedSensorReading, 0, 1023, 0, mMaxSpeedLimit);
    if (mSpeed > (tmpSpeed + 10) || mSpeed < (tmpSpeed - 10)) {
      mSpeed = tmpSpeed;
      mStepper.setMaxSpeed(mSpeed);
      Serial.print("Changing speed to: ");
      Serial.println(mSpeed);
    }

    int accelSensorReading = analogRead(accelAdjustmentPin);
    // map it to a range from 0 to 100:
    int tmpAcceleration = map(accelSensorReading, 0, 1023, 100, mMaxAcceleration);
    if (mAcceleration > (tmpAcceleration + 10) || mAcceleration < (tmpAcceleration - 10)) {
      mAcceleration = tmpAcceleration;
      mStepper.setAcceleration(mAcceleration);
      Serial.print("Changing acceleration to:");
      Serial.println(mAcceleration);
    }

    float mSpeed = mStepper.speed();
    Serial.print(mSpeed);
    Serial.print("  ");
    Serial.println(mStepper.currentPosition());
  }
}

void controller::run()
{
  handleButtons();
  handleLimitSwitches();
  mStateMachine.execute();
}

void controller::updateTargetTowardsCenter(unsigned long int steps)
{
  long int currentPos = mStepper.currentPosition();
  long int distanceToNeg = (currentPos - mNegativeLimit);
  long int distanceToPos = (mPositiveLimit - currentPos);
  if(distanceToNeg < distanceToPos)
  {
    mTargetPos + steps; // If the distance to negative limit is larger than the distance to positive limit, then the center is towards the positive limit.
  }
  else
  {
    mTargetPos - steps;
  }

}

void controller::updateTargetToOtherSide()
{
    if(mTargetPos == mPositiveLimit)
    {
      mTargetPos = mNegativeLimit;
    }
    else
    {
      mTargetPos = mPositiveLimit;
    }
}

bool controller::entryDelay()
{
  handleKnobs();
  if(mFirstEntryDelay)
  {
    mEntryTimer = 0;
    mFirstEntryDelay = false;
  }
  else if(mEntryTimer >= 1000)
  {
    mEntryTimer = 0;
    mFirstEntryDelay = true;
    return true;
  }
  return false;
}

void controller::randomPosition()
{
  mTargetPos = random(mNegativeLimit, mPositiveLimit);
}

void controller::randomSpeed()
{
  mSpeed = random(100, mMaxSpeedLimit);
}

void controller::randomAccl()
{
  mAcceleration = random(40, mMaxAcceleration);
}

bool controller::doNothingEntry()
{
  mStepper.enableOutputs();
  return false;
}

bool controller::doNothingRun()
{
  return true;
}

bool controller::doNothingExit()
{
  Serial.println("doNothingExit");
  mStepper.disableOutputs();
  return false;
}


bool controller::stopEntry()
{
  Serial.println("StopEntry");
  return false;
}


bool controller::initEntry()
{
  if(entryDelay())
  {
    Serial.println("initEntry!");
    mInitFoundFirst = false;
    mStepper.moveTo(-mMaxDistance);
    mStepper.enableOutputs();
    return false;
  }
  return true;
}
bool controller::initRun()
{
  if(!mStepper.run()) 
  {
    if(!mInitFoundFirst)
    {
      mInitFoundFirst = true;
      mStepper.moveTo(mMaxDistance);
    }
    else
    {
      mStepper.moveTo(mPositiveLimit); // a bit away from the positve limit switch
    }
  }
  return true;
}



bool controller::linearEntry()
{
  if(entryDelay())
  {
    mTargetPos = mPositiveLimit;
    mStepper.moveTo(mTargetPos);
    mStepper.enableOutputs();
    return false;
  }
  return true;
}
bool controller::linearRun()
{
  if(!mStepper.run()) 
  {
    updateTargetToOtherSide();
    mStepper.moveTo(mTargetPos);
  }
  return true;
}

bool controller::linearCompEntry()
{
  if(!mLinearCompInit)
  {
    if(entryDelay()) // first mandatory delay.
    {
      mTargetPos = ((mPositiveLimit - mNegativeLimit)/2) + mNegativeLimit;
      mStepper.moveTo(mTargetPos);
      mStepper.enableOutputs();
      Serial.print("Comp entry. moving towards center: ");
      Serial.println(mTargetPos);
      mHitRegistered = false;
      mLinearCompInit = true;
    }
  }
  else
  {
    if(!mStepper.run()) // then run to center.
    {
      mStepper.disableOutputs();
      if(mHitRegistered) // wait for first hit.
      {
        mStepper.enableOutputs();
        Serial.println("Comp entry. Center found.");
        mTargetPos = mNegativeLimit;
        mStepper.moveTo(mTargetPos);

        mLinearCompInit = false;
        mHitRegistered = false;
        mLinearCompState = LinearComp::RUN;
        return false;
      }
    }
  }
  return true;
}

bool controller::linearCompRun()
{
  switch (mLinearCompState) 
  {
    case LinearComp::RUN:
    {
      if(mHitRegistered)
      {
        Serial.print("Hit!");
        updateTargetToOtherSide();
        mStepper.moveTo(mTargetPos);
        mHitRegistered = false;
      }
      if(!mStepper.run()) 
      {
        Serial.print("Won!");
        mLinearCompState = LinearComp::WON;
      }

      break;
    }
    case LinearComp::WON:
    {
      updateTargetTowardsCenter(100);
      if(!mStepper.run())
      {
        mLinearCompState = LinearComp::CELEBRATE; // Change to celebration mode.

        // Initialize celebration mode.
        long int currentPos = mStepper.currentPosition();
        mTargetPos = currentPos + mDelta;
        mDelta = -mDelta;
        mStepper.moveTo(mTargetPos);
        mStepper.setAcceleration(10000);  // Shake violently! :)
        mStepper.setSpeed(600);
      }
      break;
    }
    case LinearComp::CELEBRATE:
    {
      if(!mStepper.run())
      {
        if(mShakes < 10)
        {
          mTargetPos += mDelta;
          mStepper.moveTo(mTargetPos);
          mDelta = -mDelta;
          mShakes++;
        }
        else
        {
          mShakes = 0;
          mStepper.setAcceleration(mAcceleration);  // reset.
          mStepper.setSpeed(mSpeed);
          mLinearCompState = LinearComp::RUN; // Reset the competetion.
          mStateMachine.changeState(states::LINEAR_COMPETITION);// restart competition. TODO map this in a better way.
        }
      }
      break;
    }
  }
  return true;
}


bool controller::randomEntry()
{
{
  if(entryDelay())
  {
    randomPosition();
    randomSpeed();
    randomAccl();
    mStepper.moveTo(mTargetPos);
    mStepper.enableOutputs();
    mHitRegistered = false;
    return false;
  }
  return true;
}
}
bool controller::randomRun()
{
  if(mHitRegistered)
  {
    mStepper.setAcceleration(10000);  // Stopp quickly, set acceleration to a high level!
    mStepper.stop();
    while (mStepper.run())
    {

    }                                         // stop as fast as possible. Let accelStepper lib stop the motor.
    mStepper.setAcceleration(mAcceleration);  // reset the acceleration so that function can resume.
    //mStepper.disableOutputs();               // then disable outputs.
    mStepper.moveTo(mStepper.currentPosition()); // move to current position, this means stop here and wait for new command.
    Serial.print("Hit,!");
    mHitRegistered = false;
  }
  else if(!mStepper.run()) 
  {
    randomPosition();
    randomSpeed();
    randomAccl();
    mStepper.moveTo(mTargetPos);
  }
  
  return true;
}


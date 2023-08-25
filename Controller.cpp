#include "variant.h"
#include "Controller.h"
#include <Wire.h>


volatile uint8_t modeButtonPressEvent = LOW;
volatile uint8_t hitButtonPressEvent = LOW;
volatile uint8_t stopButtonPressEvent = LOW;
volatile uint8_t LeftLimitSwitchEvent = LOW;
volatile uint8_t RightLimitSwitchEvent = LOW;

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels

#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32


const char* stringStates::enumtext[] = { "STOP      ", "INIT      ", "LINEAR    ", "LIN COMP  ", "RANDOM    " };


void ISR_modeButton() {
  modeButtonPressEvent = HIGH;
}
void ISR_hitButton() {
  hitButtonPressEvent = HIGH;
}
void ISR_stopButton() {
  stopButtonPressEvent = HIGH;
}

void ISR_LeftLimitSwitch() {
  LeftLimitSwitchEvent = HIGH;
}

void ISR_RightLimitSwitch() {
  RightLimitSwitchEvent = HIGH;
}

controller::controller()
  : mStepper(AccelStepper::DRIVER, stepPin, dirPin),
    mModeButton(modePin, ISR_modeButton, &modeButtonPressEvent, "mode"),
    mHitButton(hitPin, ISR_hitButton, &hitButtonPressEvent, "hit"),
    mStopButton(stopPin, ISR_stopButton, &stopButtonPressEvent, "stop"),
    mPositiveLimitSwitch(negativeLimitSwitchPin, ISR_LeftLimitSwitch, &LeftLimitSwitchEvent, "LeftLimit"),
    mNegativeLimitSwitch(positiveLimitSwitchPin, ISR_RightLimitSwitch, &RightLimitSwitchEvent, "RightLimit"),
    display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET),
    mStateMachine() {
  mStepper.setPinsInverted(false, false, true);
  mStepper.setEnablePin(enablePin);
  mStepper.setMaxSpeed(mSpeed);
  mStepper.setAcceleration(mAcceleration);
  mStepper.disableOutputs();
  mStateMachine.registerState("STOP", std::bind(&controller::stopEntry, this), std::bind(&controller::doNothingRun, this), std::bind(&controller::genericExit, this));
  mStateMachine.registerState("INIT", std::bind(&controller::initEntry, this), std::bind(&controller::initRun, this), std::bind(&controller::initExit, this));
  mStateMachine.registerState("LINEAR", std::bind(&controller::linearEntry, this), std::bind(&controller::linearRun, this), std::bind(&controller::genericExit, this));
  mStateMachine.registerState("LINEAR_COMPETITION", std::bind(&controller::linearCompEntry, this), std::bind(&controller::linearCompRun, this), std::bind(&controller::genericExit, this));
  mStateMachine.registerState("RANDOM", std::bind(&controller::randomEntry, this), std::bind(&controller::randomRun, this), std::bind(&controller::genericExit, this));
  mStateMachine.registerChangeCb(std::bind(&controller::changeCB, this, std::placeholders::_1));
};

void controller::init() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 9);              // Start at top-left corner
  display.println(F("Target!"));
  display.display();
  randomSeed(analogRead(randSeedPin));
  mStepper.disableOutputs();

  pinMode(MS2, OUTPUT);
  pinMode(MS1, OUTPUT);


  digitalWrite(MS1, LOW);  // Qarter step!
  digitalWrite(MS2, HIGH);

  delay(500);
}

void controller::changeCB(int state) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 9);              // Start at top-left corner
  display.print(stringStates::enumtext[state]);
  display.display();
}


void controller::handleLimitSwitches() {
  bool positivePressed = mPositiveLimitSwitch.pressed();
  bool negativePressed = mNegativeLimitSwitch.pressed();

  if (positivePressed || negativePressed) {

    mStepper.setAcceleration(mStopFastAcc * 2);  // Stopp quickly, set acceleration to a high level!
    mStepper.stop();
    while (mStepper.run()) {

    }                                         // stop as fast as possible. Let accelStepper lib stop the motor.
    mStepper.setAcceleration(mAcceleration);  // reset the acceleration so that function can resume.
    //mStepper.disableOutputs();               // then disable outputs.
    mStepper.moveTo(mStepper.currentPosition());  // move to current position, this means stop here and wait for new command.

    if (positivePressed) {
      mPositiveLimit = mStepper.currentPosition() - mMarginFromEnd;
      Serial.print("positive limit reached: ");
      Serial.println(mPositiveLimit);

    } else {
      mNegativeLimit = mStepper.currentPosition() + mMarginFromEnd;
      Serial.print("negative limit reached: ");
      Serial.println(mNegativeLimit);
    }
  }
}
void controller::handleButtons() {
  if (mModeButton.pressed()) {
    mFirstEntryDelay = true;
    mStateMachine.changeState();
  }
  if (mHitButton.pressed()) {
    mHitRegistered = true;
  }
  if (mStopButton.pressed()) {
    mStateMachine.changeState(states::STOP);
  }
}

void controller::handleKnobs() {

  int speedSensorReading = analogRead(speedAdjustmentPin);
  // map it to a range from 0 to 100:
  int tmpSpeed = map(speedSensorReading, 0, 1023, 0, mMaxSpeedLimit);
  mSpeed = tmpSpeed;
  mStepper.setMaxSpeed(mSpeed);
  Serial.print("Changing speed to: ");
  Serial.println(mSpeed);

  int accelSensorReading = analogRead(accelAdjustmentPin);
  // map it to a range from 0 to 100:
  int tmpAcc = map(accelSensorReading, 0, 1023, 100, mMaxAcceleration);
  mAcceleration = tmpAcc;
  mStepper.setAcceleration(mAcceleration);
  Serial.print("Changing acceleration to:");
  Serial.println(mAcceleration);
}

void controller::run() {
  handleButtons();
  handleLimitSwitches();
  mStateMachine.execute();
}

void controller::updateTargetTowardsCenter(unsigned long int steps) {
  long int currentPos = mStepper.currentPosition();
  long int distanceToNeg = (currentPos - mNegativeLimit);
  long int distanceToPos = (mPositiveLimit - currentPos);
  if (distanceToNeg < distanceToPos) {
    mTargetPos + steps;  // If the distance to negative limit is larger than the distance to positive limit, then the center is towards the positive limit.
  } else {
    mTargetPos - steps;
  }
}

void controller::updateTargetToOtherSide() {
  if (mTargetPos == mPositiveLimit) {
    mTargetPos = mNegativeLimit;
  } else {
    mTargetPos = mPositiveLimit;
  }
}

bool controller::entryDelay() {
  handleKnobs();
  if (mFirstEntryDelay) {
    mEntryTimer = 0;
    mFirstEntryDelay = false;
  } else if (mEntryTimer >= mInitDelay) {
    mEntryTimer = 0;
    mFirstEntryDelay = true;
    return true;
  }
  return false;
}

void controller::randomPosition() {
  mTargetPos = random(mNegativeLimit, mPositiveLimit);
}

void controller::randomSpeed() {
  mSpeed = random(mMaxSpeedLimit / 4, mAcceleration);
  mStepper.setMaxSpeed(mSpeed);
}

void controller::randomAccl() {
  mAcceleration = random(mMaxAcceleration / 4, mMaxAcceleration);
  mStepper.setAcceleration(mAcceleration);
}

bool controller::randomDelay() {
  if (mFirstRandDelay) {
    mRandDelay = random(0, mMaxDelay);
    mRandTimer = 0;
    mFirstRandDelay = false;
  } else if (mRandTimer >= mRandDelay) {
    mRandTimer = 0;
    mFirstRandDelay = true;
    return true;
  }
  return false;
}

bool controller::doNothingEntry() {
  mStepper.enableOutputs();
  return false;
}

bool controller::doNothingRun() {
  return true;
}

bool controller::doNothingExit() {
  mStepper.disableOutputs();
  return false;
}

bool controller::genericExit() {

  mStepper.setAcceleration(mStopFastAcc);  // Stopp quickly, set acceleration to a high level!
  mStepper.stop();
  while (mStepper.run()) {
  }
  mStepper.moveTo(mStepper.currentPosition());  // move to current position, this means stop here and wait for new command.
  mStepper.setAcceleration(mAcceleration);
  mStepper.disableOutputs();
  return false;
}

bool controller::stopEntry() {
  mStepper.moveTo(mStepper.currentPosition());
  mStepper.disableOutputs();
  return false;
}


bool controller::initEntry() {
  if (entryDelay()) {

    mInitFoundFirst = false;
    mStepper.setCurrentPosition(0);
    mStepper.moveTo(-mMaxDistance);
    mStepper.enableOutputs();
    mStepper.setMaxSpeed(150 * stepMultiplier);
    return false;
  }
  return true;
}

bool controller::initRun() {
  if (!mStepper.run()) {
    if (!mInitFoundFirst) {
      mInitFoundFirst = true;
      mStepper.moveTo(mMaxDistance);
    } else {
      mStepper.moveTo(mPositiveLimit);  // a bit away from the positve limit switch
      while (mStepper.run()) {
      }
      mStateMachine.changeState(states::STOP);
    }
  }
  return true;
}

bool controller::initExit() {
  Serial.print("positive limit : ");
  Serial.println(mPositiveLimit);
  Serial.print("negative limit : ");
  Serial.println(mNegativeLimit);
  return false;
}


bool controller::linearEntry() {
  if (entryDelay()) {
    mTargetPos = mPositiveLimit;
    mStepper.moveTo(mTargetPos);
    mStepper.enableOutputs();
    return false;
  }
  return true;
}
bool controller::linearRun() {
  if (!mStepper.run()) {
    updateTargetToOtherSide();
    mStepper.moveTo(mTargetPos);
  }
  return true;
}

bool controller::linearCompEntry() {
  if (!mLinearCompInit) {
    if (entryDelay())  // first mandatory delay.
    {
      mTargetPos = ((mPositiveLimit - mNegativeLimit) / 2) + mNegativeLimit;
      mStepper.moveTo(mTargetPos);
      mStepper.enableOutputs();
      Serial.println(mTargetPos);
      mHitRegistered = false;
      mLinearCompInit = true;
    }
  } else {
    if (!mStepper.run())  // then run to center.
    {
      mStepper.disableOutputs();
      if (mHitRegistered)  // wait for first hit.
      {
        mStepper.enableOutputs();
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

bool controller::linearCompRun() {
  switch (mLinearCompState) {
    case LinearComp::RUN:
      {
        if (mHitRegistered) {
          Serial.print("Hit!");
          mStepper.setAcceleration(mStopFastAcc);  // Stopp quickly, set acceleration to a high level!
          mStepper.stop();
          while (mStepper.run()) 
          {

          }                                         // stop as fast as possible. Let accelStepper lib stop the motor.
          mStepper.setAcceleration(mAcceleration);  // reset the acceleration so that function can resume.
          updateTargetToOtherSide();
          mStepper.moveTo(mTargetPos);
          mHitRegistered = false;
        }
        if (!mStepper.run()) {
          Serial.print("Won!");
          mLinearCompState = LinearComp::WON;
        }

        break;
      }
    case LinearComp::WON:
      {
        updateTargetTowardsCenter(100);
        if (!mStepper.run()) {
          mLinearCompState = LinearComp::CELEBRATE;  // Change to celebration mode.

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
        if (!mStepper.run()) {
          if (mShakes < 10) {
            mTargetPos += mDelta;
            mStepper.moveTo(mTargetPos);
            mDelta = -mDelta;
            mShakes++;
          } else {
            mShakes = 0;
            mStepper.setAcceleration(mAcceleration);  // reset.
            mStepper.setSpeed(mSpeed);
            mLinearCompState = LinearComp::RUN;                     // Reset the competetion.
            mStateMachine.changeState(states::LINEAR_COMPETITION);  // restart competition. TODO map this in a better way.
          }
        }
        break;
      }
  }
  return true;
}


bool controller::randomEntry() {
  if (entryDelay()) {
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


bool controller::randomRun() {
  if (mHitRegistered) {
    mStepper.setAcceleration(mStopFastAcc);  // Stopp quickly, set acceleration to a high level!
    mStepper.stop();
    while (mStepper.run()) {

    }                                             // stop as fast as possible. Let accelStepper lib stop the motor.
    mStepper.setAcceleration(mAcceleration);      // reset the acceleration so that function can resume.
    mStepper.moveTo(mStepper.currentPosition());  // move to current position, this means stop here and wait for new command.
    mHitRegistered = false;
  } else if (!mStepper.run()) {
    if (randomDelay()) {
      randomPosition();
      randomSpeed();
      randomAccl();
      mStepper.moveTo(mTargetPos);
    }
  }

  return true;
}

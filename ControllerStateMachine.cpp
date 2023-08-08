#include "ControllerStateMachine.h"

namespace std {
    void __throw_bad_function_call() {}
}

StateMachine::StateMachine(): mStates(), mStage(stage::entry), mCurrentState(0), mNextState(0)
{ 

}

void StateMachine::execute() 
{
  if (mCurrentState <= mStates.size()) 
  {
    switch (mStage) 
    {
      case stage::entry:
        if(mStates[mCurrentState].mEntry)
        {
          if(!mStates[mCurrentState].mEntry()) // Change state once entry is finished.
          {
            mStage = stage::run; 
          } 
        }
        break;
      case stage::run:
        if(mStates[mCurrentState].mRun)
        {
          if(!mStates[mCurrentState].mRun())
          {
            //error...
          }
        }
        break;
      case stage::exit:
        if(mStates[mCurrentState].mExit)
        {
          if(!mStates[mCurrentState].mExit()) // Change state once exit is finished.
          {
            // After exiting the stage, move to the next state, begin with entry stage.
            mStage = stage::entry;
            Serial.println("State is: " + mStates[mCurrentState].mName);

            mCurrentState = mNextState;
            Serial.println("State changed to: " + mStates[mCurrentState].mName);
          }

        }
        break;
    }
  }
  else
  {
    //Error..
  }
}

void StateMachine::changeState(int state)
{
  mStage = stage::exit; // This will cause execute to enter the exit stage, and move to the next state.

  if(state == 0xFFFF)
  {
    String nextName = mStates[mNextState].mName;
    Serial.print("NextState is: " + nextName + " ");
    Serial.println(mNextState);

    mNextState++;
    mNextState %= mStates.size(); // wrap around.

    nextName = mStates[mNextState].mName;
    Serial.print("NextState is: " + nextName + " ");
    Serial.println(mNextState);

    String name = mStates[mCurrentState].mName;
    Serial.print("Current state: " + name + " ");
    Serial.println(mCurrentState);
  }
  else if(state < mStates.size())
  {
    mNextState = state;
  }

}  

void StateMachine::registerState(String name, StateMashineFunction entry, StateMashineFunction run, StateMashineFunction exit)
{
  mStates.push_back(State(name, entry, run, exit));
}

int StateMachine::getNrRegisteredStates()
{
  return mStates.size();
}


StateMachine::State::State(String name, StateMashineFunction entry, StateMashineFunction run, StateMashineFunction exit): mName(name), mEntry(entry), mRun(run), mExit(exit)
{

}


#ifndef CONTROLLER_STATE_MACHINE_INCLUDE_GUARD
#define CONTROLLER_STATE_MACHINE_INCLUDE_GUARD

#include <vector>
#include <functional>
#include <Arduino.h>

typedef std::function<bool()> StateMashineFunction;

typedef std::function<void(int)> StateChangeCb;


class StateMachine 
{
public:
  StateMachine();

  void execute(); //execute the selected state.

  void changeState(int state = 0xFFFF);

  void registerState(String name, StateMashineFunction entry, StateMashineFunction run, StateMashineFunction exit);

  void registerChangeCb(StateChangeCb cb);
  int getNrRegisteredStates();
  private:
  
  enum stage {
    entry,
    run,
    exit
  };

  class State
  {
    public:
    State(String name, StateMashineFunction entry, StateMashineFunction run, StateMashineFunction exit);
    String mName;
    StateMashineFunction mEntry;
    StateMashineFunction mRun;
    StateMashineFunction mExit;
  };

  StateChangeCb mChangeCB;
  std::vector<State> mStates;
  int mCurrentState = 0;
  int mNextState = 0;
  stage mStage = stage::entry;
  bool mStarted;
};

#endif /* CONTROLLER_STATE_MACHINE_INCLUDE_GUARD */

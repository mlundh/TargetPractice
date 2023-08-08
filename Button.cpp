#include "Button.h"
 
 Button::Button(uint32_t pin, void(*isr)(), volatile uint8_t* buttonEvent, String name) :
    startOfEvent(0), buttonState(LOW), debounceTimeMs(10), buttonEvent(buttonEvent), pin(pin), name(name)
  { 
    pinMode(pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pin), isr, FALLING);
  }

  Button::~Button()
  {

  }

  bool Button::pressed()
  {
    bool acceptedPress = false;
    if(*buttonEvent)
    {
      uint32_t time = millis();
      if(buttonState == LOW)// If this is the first event, record time and wait debounce time to determine if it is a good press.
      {
        startOfEvent = time;
        buttonState = HIGH;
      }
      if((time - startOfEvent) > debounceTimeMs) // Wait debounceTimeMs and then check input state.
      {
        if(digitalRead(pin) == LOW) // if the line is still low, then it is an OK press, otherwise discard.
        {
          acceptedPress = true;
        }
        *buttonEvent = LOW; // Clear button event in preparation for next event.
        buttonState = LOW;  // Clear button state in preparation for next possible press.
      }
    }
    return acceptedPress;
  }
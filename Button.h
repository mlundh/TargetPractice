
#ifndef BUTTON_INCLUDE_GUARD
#define BUTTON_INCLUDE_GUARD
 
#include <Arduino.h>

class Button
{
private:
  uint32_t startOfEvent;
  uint8_t buttonState;
  const uint32_t debounceTimeMs;
  volatile uint8_t* buttonEvent;
  uint32_t pin;
  String name;
  Button();// not allowed to construct without a buttion event.

public:
  Button(uint32_t pin, void(*isr)(), volatile uint8_t* buttonEvent, String name);

  ~Button();

  bool pressed();
};
#endif /* BUTTON_INCLUDE_GUARD */

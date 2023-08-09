#include "Controller.h"


controller myController;


int32_t goToPos = 1500;

void setup() 
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT); 
  randomSeed(analogRead(A1));
  myController.init();
}


bool stop = true;

void loop() 
{  
  myController.run();
}

#include <AccelStepper.h>
#include "step.h"

void setup() {
}

void loop() {

  actuateBottom();
  actuateTop();
  delay(6000);
}

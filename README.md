# cutdown
This repo holds software for the LASP high altitude balloon cutdown controller's primary microcontroller (ATSAMD21G18A).It is in the form of an Arduino library since the board is programmed as if it is an Adafruit Feather M0.

A simple arduino file such as the following can run the library and the board:
```c++
#include <Cutdown.h>

Cutdown cutdown;

void setup() {
  Serial.begin(115200);
  cutdown.init();
}

void loop() {
  cutdown.run();
}
```

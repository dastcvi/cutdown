# cutdown
This repo holds software for the LASP high altitude balloon cutdown controller's primary microcontroller (ATSAMD21G18A). It is in the form of an Arduino library since the board is programmed as if it is an Adafruit Feather M0.

## Hardware
There are two hardware revisions that this library currently supports. Both can be found in [this repository](https://github.com/dastcvi/cutdown_hardware) as releases.

## Git branch strategy
The master branch is reserved for notable releases and also contains early development. There are unique branches for each hardware in order to ensure that earlier revisions can still be used for testing. These branches are also tagged with notable releases.

## Running the library through Arduino
The library is designed to avoid using the Arduino IDE as much as possible, but Arduino is still used for the compiler, libraries, and loader. The Adafruit Feather M0 bootloader must be loaded onto the board, which can be done following [this tutorial](https://learn.adafruit.com/proper-step-debugging-atsamd21-arduino-zero-m0/overview). Once this is done, the board can be programmed over USB using the Arduino IDE. The correct board must be selected: `Tools > Board: Adafruit Feather M0`. A simple arduino file such as the following can run the library and the board:
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

### Examples directory
The examples directory holds the scripts that use the library. This includes the main script used to run the board as well as test scripts used while developing certain components.

#include "mbed.h"
#include "RoverControl.hpp"

// main() runs in its own thread in the OS
int main()
{
    leftForwardControl = 0;
    leftBackwardControl = 0;
    rightForwardControl = 0;
    rightBackwardControl = 0;
    while (true) {
        forward(10000000, 0.5);
        reverse(10000000, 0.5);
    }
}


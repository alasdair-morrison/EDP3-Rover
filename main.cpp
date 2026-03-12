#undef __ARM_FP
#include "mbed.h"
//Left Motor Pins
PwmOut leftMotor(PTA1);
DigitalOut leftForwardControl(PTA2);
DigitalOut leftBackwardControl(PTD4);
//Right Motor Pins
PwmOut rightMotor(PTA12);
DigitalOut rightForwardControl(PTA5);
DigitalOut rightBackwardControl(PTA4);
//Detection LED
PwmOut powerLED(LED_RED);

//IR Sensor Pins
DigitalIn leftIR(PTD5);
DigitalIn middleIR(PTD0);
DigitalIn rightIR(PTD2);
DigitalIn rightTurnIR(PTD3);
DigitalIn leftTurnIR(PTA13);

//Ultrasonic Sensor Pins
DigitalOut frontTrigger(PTD1);
DigitalIn frontEcho(PTE0);
DigitalOut sideTrigger();
DigitalIn sideEcho();

void forward(float duty) {
    leftMotor.write(duty);
    rightMotor.write(duty);

    leftBackwardControl = 0;
    rightBackwardControl = 0;
    leftForwardControl  = 1;
    rightForwardControl = 1;
}

void reverse(float duty) {
    leftMotor.write(duty);
    rightMotor.write(duty);

    leftForwardControl  = 0;
    rightForwardControl = 0;
    leftBackwardControl = 1;
    rightBackwardControl = 1;
}

void turnLeft(float duty) {
    leftMotor.write(duty);
    rightMotor.write(duty);

    leftForwardControl  = 0;
    rightForwardControl = 1;
    leftBackwardControl = 1;
    rightBackwardControl = 0;
    wait_us(10000);

}

void turnRight(float duty) {
    leftMotor.write(duty);
    rightMotor.write(duty);

    leftForwardControl  = 1;
    rightForwardControl = 0;
    leftBackwardControl = 0;
    rightBackwardControl = 1;
    wait_us(10000);

}

void stop() {
    leftMotor.write(0.0f);
    rightMotor.write(0.0f);

    leftForwardControl  = 0;
    leftBackwardControl = 0;
    rightForwardControl = 0;
    rightBackwardControl = 0;
}

void fullStop() {
    leftMotor.write(0.8f);
    rightMotor.write(0.8f);

    leftForwardControl  = 1;
    leftBackwardControl = 1;
    rightForwardControl = 1;
    rightBackwardControl = 1;
}

void cornerLeft(float dutyTurnRight) {
    turnLeft(dutyTurnRight);
        wait_us(40000);

}

void cornerRight(float dutyTurnLeft) {
    turnRight(dutyTurnLeft);
        wait_us(40000);

}

int FSM = 0;
Timer High_Time;
Timer Emergency_Timer;
double Echo_High_Time = 0.0;
double Object_Distance = 0.0;

void avoidObstacle() {
    stop();
    wait_us(5000);
}

void ultrasonicHandler() {
    if (FSM == 0 && frontEcho == 0) {
            frontTrigger = 0;
            wait_us(1);   // Just a clean transition start
            frontTrigger = 1;
            wait_us(10);  // Set Trigger to High for 10 us
            frontTrigger = 0;

            Emergency_Timer.reset();     // we neeed thhis in case trigger goes of but echo does not due to a loose connection or any reason
            Emergency_Timer.start();

            FSM = 1;   // Finite-State Machine is on mode 1
        }

        if (FSM == 1 && frontEcho == 1) {
            High_Time.reset();
            High_Time.start();

            Emergency_Timer.stop();     // end the timer since if we get to this point we cant be stuck anymore

            FSM = 2;   // Finite-State Machine is on mode 2
        }

        if (FSM == 2 && frontEcho == 0) {
            High_Time.stop();
            FSM = 3;   // Finite-State Machine is on mode 2
        }    

        if (FSM == 3 && frontEcho == 0) {
            // Calculate time in us
            Echo_High_Time = High_Time.elapsed_time().count();
            // CONVERTED TO cm/us form m/s from datasheet
            Object_Distance = (Echo_High_Time * (0.034/ 2.0));
            FSM = 0;   // Finite-State Machine is on mode 2
        }    

        float Emergency_Time = Emergency_Timer.elapsed_time().count();


        if ((FSM == 1 || FSM == 2) && Emergency_Time > 60000.0){
            Emergency_Timer.stop();     // we neeed this in case trigger goes of but echo does not due to a loose connection or any reason
            Emergency_Timer.reset();
            FSM = 0; 
        } // Prevent being stuck if wire is loose and echo never goes high
}

int main()
{
    stop();
    float period = 1.0/40000;
    float duty = 0.6;
    float dutyTurnRight = 0.8767;
    float dutyTurnLeft = 0.6767;
    leftMotor.period(period);
    rightMotor.period(period);

    while (true) {

        ultrasonicHandler();
        
        /*cm and if 20 cm then 5882.35 us time to bounce back*/
        if (Object_Distance <= 20.0) {
            avoidObstacle();
        }
        int leftValue = leftIR.read();
        int rightValue = rightIR.read();
        int leftTurnValue = leftTurnIR.read();
        int rightTurnValue = rightTurnIR.read();
        int middleValue = middleIR.read();

        // If 90 degree right turn is needed
        if ((leftValue == 1 && rightValue == 1 && rightTurnValue == 1 && middleValue == 1 && leftTurnValue == 0) || (leftTurnValue == 0 && leftValue == 0 && middleValue == 1 && rightValue == 1 && rightTurnValue == 1 )) {
            cornerRight(dutyTurnRight);
        }
        // If 90 degree left turn is needed
        if ((leftValue == 1 && rightValue == 1 && rightTurnValue == 0 && middleValue == 1 && leftTurnValue == 1) || (leftTurnValue == 1 && leftValue == 0 && middleValue == 1 && rightValue == 0 && rightTurnValue == 0 )) {
            cornerLeft(dutyTurnLeft);
        }
        // If both sensors are on BLACK, move forward
        else if (leftValue == 1 && rightValue == 1 && middleValue == 1 && leftTurnValue == 0 && rightTurnValue == 0) {
            forward(duty - 0.3);
        }
        // Left sensor on black line, turn left
        else if ( (leftTurnValue == 0 && leftValue == 0 &&  middleValue == 1 && rightValue == 1  &&  rightTurnValue == 0 )|| (leftTurnValue == 1 && leftValue == 1)) {
            turnLeft(duty);
        }
        // Right sensor on black line, turn right
        else if ( (leftTurnValue == 0 && leftValue == 0 &&  middleValue == 1 && rightValue == 1  &&  rightTurnValue == 0 )|| (rightTurnValue == 1 && rightValue == 1)) {
            turnRight(duty);
        }
             // Both sensors on WHITE, stop
        else if (leftValue == 1 && rightValue == 1 && middleValue == 1 && rightTurnValue == 1 && leftTurnValue == 1) {
            fullStop();
        }
    }
}
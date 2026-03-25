#undef __ARM_FP
#include "mbed.h"
#include "TCS3200.h"
#include <list>
#include <numeric>

//Left Motor Pins
PwmOut leftMotor(PTD4);
DigitalOut leftForwardControl(PTA12);
DigitalOut leftBackwardControl(PTA4);
//Right Motor Pins
PwmOut rightMotor(PTA5);
DigitalOut rightForwardControl(PTC9);
DigitalOut rightBackwardControl(PTC8);

//IR Sensor Pins
DigitalIn leftIR(PTD0);
DigitalIn middleIR(PTD2);
DigitalIn rightIR(PTD3);
DigitalIn rightTurnIR(PTD1);
DigitalIn leftTurnIR(PTD5);

//Ultrasonic Sensor Pins
DigitalOut frontTrigger(PTE0);
DigitalIn frontEcho(PTE1);
DigitalOut sideTrigger(PTB1);
DigitalIn sideEcho(PTB0);

DigitalOut red_led(LED_RED);
// Pin configuration: S0, S1, S2, S3, OUT
TCS3200 colourSensor(PTC1, PTC2, PTB3, PTB2, PTA13);
static BufferedSerial pc(USBTX, USBRX, 9600);
float avgRed = 0;
float avgGreen = 0;
float avgBlue = 0;

float period = 1.0/40000;
float duty = 0.6;
float dutyTurnRight = 0.8;
float dutyTurnLeft = 0.8;

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
}

void turnRight(float duty) {
    leftMotor.write(duty);
    rightMotor.write(duty);

    leftForwardControl  = 1;
    rightForwardControl = 0;
    leftBackwardControl = 0;
    rightBackwardControl = 1;
}

void stop() {
    leftMotor.write(0.0f);
    rightMotor.write(0.0f);

    leftForwardControl  = 0;
    leftBackwardControl = 0;
    rightForwardControl = 0;
    rightBackwardControl = 0;
}

void cornerLeft(float dutyTurnRight) {
    turnLeft(dutyTurnRight);
        wait_us(40000);

}

void cornerRight(float dutyTurnLeft) {
    turnRight(dutyTurnLeft);
        wait_us(40000);

}

void fullStop() {
    leftMotor.write(0.8f);
    rightMotor.write(0.8f);

    leftForwardControl  = 1;
    leftBackwardControl = 1;
    rightForwardControl = 1;
    rightBackwardControl = 1;
}

void steerLeft(float baseDuty) {
    // Both wheels forward
    leftForwardControl  = 1;
    rightForwardControl = 1;
    leftBackwardControl = 0;
    rightBackwardControl = 0;

    // Slow down the left wheel to arc left towards the object
    leftMotor.write(baseDuty - 0.2); 
    rightMotor.write(baseDuty);
}

void steerRight(float baseDuty) {
    // Both wheels forward
    leftForwardControl  = 1;
    rightForwardControl = 1;
    leftBackwardControl = 0;
    rightBackwardControl = 0;

    // Slow down the right wheel to arc right away from the object
    leftMotor.write(baseDuty);
    rightMotor.write(baseDuty - 0.2); 
}

int FSMF = 0;
int FSMS = 0;
Timer High_TimeF;
Timer High_TimeS;
Timer Emergency_Timer;
double Echo_High_TimeFront = 0.0;
double Echo_High_TimeSide = 0.0;

double getDistanceSide(double Object_Distance) {
    if (FSMS == 0 && sideEcho == 0) {
            sideTrigger = 0;
            wait_us(1);   // Just a clean transition start
            sideTrigger = 1;
            wait_us(10);  // Set Trigger to High for 10 us
            sideTrigger = 0;

            Emergency_Timer.reset();     // we neeed thhis in case trigger goes of but echo does not due to a loose connection or any reason
            Emergency_Timer.start();

            FSMS = 1;   // Finite-State Machine is on mode 1
        }

        if (FSMS == 1 && sideEcho == 1) {
            High_TimeS.reset();
            High_TimeS.start();

            Emergency_Timer.stop();     // end the timer since if we get to this point we cant be stuck anymore

            FSMS = 2;   // Finite-State Machine is on mode 2
        }

        if (FSMS == 2 && sideEcho == 0) {
            High_TimeS.stop();
            FSMS = 3;   // Finite-State Machine is on mode 2
        }    

        if (FSMS == 3 && sideEcho == 0) {
            // Calculate time in us
            Echo_High_TimeSide = High_TimeS.elapsed_time().count();
            // CONVERTED TO cm/us form m/s from datasheet
            Object_Distance = (Echo_High_TimeSide * (0.034/ 2.0));
            FSMS = 0;   // Finite-State Machine is on mode 2
        }    

        float Emergency_Time = Emergency_Timer.elapsed_time().count();


        if ((FSMS == 1 || FSMS == 2) && Emergency_Time > 60000.0){
            Emergency_Timer.stop();     // we neeed this in case trigger goes of but echo does not due to a loose connection or any reason
            Emergency_Timer.reset();
            FSMS = 0; 
        } // Prevent being stuck if wire is loose and echo never goes high
    return Object_Distance;
}

double ultrasonicHandlerFront(double Object_Distance) {
    if (FSMF == 0 && frontEcho == 0) {
            frontTrigger = 0;
            wait_us(1);   // Just a clean transition start
            frontTrigger = 1;
            wait_us(10);  // Set Trigger to High for 10 us
            frontTrigger = 0;

            Emergency_Timer.reset();     // we neeed thhis in case trigger goes of but echo does not due to a loose connection or any reason
            Emergency_Timer.start();

            FSMF = 1;   // Finite-State Machine is on mode 1
        }

        if (FSMF == 1 && frontEcho == 1) {
            High_TimeF.reset();
            High_TimeF.start();

            Emergency_Timer.stop();     // end the timer since if we get to this point we cant be stuck anymore

            FSMF = 2;   // Finite-State Machine is on mode 2
        }

        if (FSMF == 2 && frontEcho == 0) {
            High_TimeF.stop();
            FSMF = 3;   // Finite-State Machine is on mode 2
        }    

        if (FSMF == 3 && frontEcho == 0) {
            // Calculate time in us
            Echo_High_TimeFront = High_TimeF.elapsed_time().count();
            // CONVERTED TO cm/us form m/s from datasheet
            Object_Distance = (Echo_High_TimeFront * (0.034/ 2.0));
            FSMF = 0;   // Finite-State Machine is on mode 2
        }    

        float Emergency_Time = Emergency_Timer.elapsed_time().count();


        if ((FSMF == 1 || FSMF == 2) && Emergency_Time > 60000.0){
            Emergency_Timer.stop();     // we neeed this in case trigger goes of but echo does not due to a loose connection or any reason
            Emergency_Timer.reset();
            FSMF = 0; 
        } // Prevent being stuck if wire is loose and echo never goes high
        return Object_Distance;
}

bool lineDetected() {
    int leftValue = leftIR.read();
    int rightValue = rightIR.read();
    int leftTurnValue = leftTurnIR.read();
    int rightTurnValue = rightTurnIR.read();
    int middleValue = middleIR.read();
    if (leftValue == 1 || rightValue == 1 || middleValue == 1 || rightTurnValue == 1 || leftTurnValue == 1 ) {
        return true;
    }
    return false;
}

void avoidObstacle() {
    // Start with a dummy value > 20 so the loop triggers
    double sideDistance = 30.0; 

    // STEP 1: Turn Right until the side sensor "catches" the obstacle
    /*while (true) {
        sideDistance = getDistanceSide(sideDistance);
        
        // Break the loop once the side sensor gets a valid, close reading
        if (sideDistance > 0.1 && sideDistance <= 20.0) {
            break; 
        }
        turnRight(0.4);
    }*/
    cornerRight(duty);

    // STEP 2: Follow the perimeter until the line is found
    while (!lineDetected()) {
        sideDistance = getDistanceSide(sideDistance);

        // Avoid reacting to erroneous "0" readings if the sensor misfires
        if (sideDistance <= 0.1) {
            continue; 
        }

        // The "Deadband" logic: Maintain a distance between 15cm and 25cm
        if (sideDistance > 25.0) {
            // Drifting away from the obstacle -> steer toward it
            turnLeft(duty-0.3);
        } 
        else if (sideDistance < 15.0) {
            // Getting too close to the obstacle -> steer away
            turnRight(duty-0.3);
        } 
        else {
            // In the sweet spot (15cm - 25cm) -> drive straight
            forward(duty-0.5);
        }
    }
}

void avoidObstacleHard() {
    Timer orbitTimer;

    // STEP 1: The Evasion Turn
    // Turn right just enough so the robot is parallel to the paint can.
    // (You will need to tune this 400000us / 0.4s value to match your robot's speed)
    turnRight(dutyTurnRight);
    wait_us(400000); 

    // STEP 2: Lock into the Orbit
    // Set both wheels to drive forward, but make the left wheel slower.
    // This forces the robot to drive in a fixed left-handed circle.
    leftForwardControl  = 1;
    rightForwardControl = 1;
    leftBackwardControl = 0;
    rightBackwardControl = 0;

    // (Tune the 0.25f offset to change the tightness of the circle)
    leftMotor.write(duty - 0.25f); // Slower inside wheel
    rightMotor.write(duty);        // Faster outside wheel

    // STEP 3: Wait for the Line (with Failsafe)
    orbitTimer.start();
    
    // The robot will continuously drive in its arc until lineDetected() becomes true.
    // We include a 5.0 second failsafe so it doesn't orbit forever if it misses.
    while (!lineDetected() && orbitTimer.read() < 5.0) {
        // Do nothing. Just let the motors run and wait for the sensors.
    }

    // STEP 4: Cleanup
    orbitTimer.stop();
    stop(); // Pause briefly to settle before returning to the main line-following loop
    wait_us(100000); 
}

float dynamicMinGap = 50.0;
float dynamicMaxRed = 75.0;

bool redDetected() {
    // Tunable "Smoothness" (0.1 = Very Smooth/Slow, 0.9 = Fast/Jittery)
    float alpha = 0.9; 
    long rawRed = colourSensor.ReadRed();
    long rawGreen = colourSensor.ReadGreen();
    long rawBlue = colourSensor.ReadBlue();
    
    // Apply Filter
    avgRed = (rawRed * alpha) + (avgRed * (1.0 - alpha));
    avgGreen = (rawGreen * alpha) + (avgGreen * (1.0 - alpha));
    avgBlue = (rawBlue * alpha) + (avgBlue * (1.0 - alpha));
    //printf("R: %d   G: %d   B: %d\r\n", int(avgRed), int(avgGreen), int(avgBlue));
    float blueGap = avgBlue - avgRed;
    float greenGap = avgGreen - avgRed;
        
    // 5. The "Contrast" Check
    // "Is Blue 50 units weaker than Red? AND Is Green 50 units weaker?"
    if (blueGap > dynamicMinGap && greenGap > dynamicMinGap && avgRed < dynamicMaxRed) {
        red_led = 0; // Turn ON
        //printf("Red Detected\r\n");
        return true;
    } 
    else {
        red_led = 1; // Turn OFF
        //printf("No Red Detected\r\n");
        return false;
    }
}

double baselineRed() {
    long totalRed = 0;
    long totalGreen = 0;
    long totalBlue = 0;
    int numSamples = 20;

    printf("Calibrating... Keep sensor over a neutral background (white/gray).\r\n");

    // Take multiple readings to average out sensor noise
    for (int i = 0; i < numSamples; i++) {
        totalRed += colourSensor.ReadRed();
        totalGreen += colourSensor.ReadGreen();
        totalBlue += colourSensor.ReadBlue();
        wait_us(10000); // Wait 10ms between samples
    }

    // Calculate the baseline ambient values
    float ambientRed = (float)totalRed / numSamples;
    float ambientGreen = (float)totalGreen / numSamples;
    float ambientBlue = (float)totalBlue / numSamples;

    // Calculate the natural ambient gap in the current lighting
    float ambientBlueGap = ambientBlue - ambientRed;
    float ambientGreenGap = ambientGreen - ambientRed;

    // RULE 1: Adjust Min Gap
    // The required gap to detect red must be larger than the natural ambient gap.
    // We average the ambient gaps and add a buffer (e.g., 30 units) for contrast.
    dynamicMinGap = ((ambientBlueGap + ambientGreenGap) / 2.0) + 10.0;

    // RULE 2: Adjust Max Red
    // A red object will make the raw red value drop (stronger signal). 
    // We set the detection threshold to 85% of the ambient baseline.
    dynamicMaxRed = ambientRed * 0.90;

    printf("Calibration Done! New MinGap: %.2ld | New MaxRed: %.2ld\r\n", long(dynamicMinGap), long(dynamicMaxRed));

    return dynamicMaxRed;
}

int main()
{
    stop();
    leftMotor.period(period);
    rightMotor.period(period);
    double Object_Distance_Front = 30;
    colourSensor.SetMode(TCS3200::SCALE_20);
    Timer colorCheckTimer;
    colorCheckTimer.start();
    dynamicMaxRed = baselineRed();
    while (true) {
        if (colorCheckTimer.read() > 0.05) {
                while (redDetected()) {
                    fullStop();
                    wait_us(50000); // 50ms buffer so it doesn't spam reads
                }
            // Reset the timer after checking
            colorCheckTimer.reset(); 
        }
        //Object_Distance_Front = ultrasonicHandlerFront(Object_Distance_Front);
            
        /*cm and if 20 cm then 5882.35 us time to bounce back*/
        if (Object_Distance_Front <= 20.0) {
            //avoidObstacleHard();
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
            forward(duty-0.1);

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
            stop();
        }        
      //  else if (leftValue == 0 && rightValue == 0 && middleValue == 0 && rightTurnValue == 0 && leftTurnValue == 0) {
     //       reverse(duty);
     //       wait_us(20000);
     //       stop();
    //    }
    }
}
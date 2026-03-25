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

Thread colorThread(osPriorityNormal, 1024);
volatile bool isStoppedForRed = false; 

Thread ultrasonicThread(osPriorityNormal, 1024);
volatile double globalFrontDistance = 999.0;
volatile double globalSideDistance = 999.0;
volatile bool objectDetected = false;

// A universal function to read ANY ultrasonic sensor
double pingUltrasonic(DigitalOut& trigger, DigitalIn& echo) {
    Timer echoTimer;
    Timer timeoutTimer;
    
    trigger = 0;
    wait_us(2);
    trigger = 1;
    wait_us(10); // 10us pulse
    trigger = 0;
    
    timeoutTimer.start();
    
    // Wait for echo pin to go HIGH
    while (echo == 0) {
        // If it takes longer than 30ms, the pulse was lost. Timeout.
        if (timeoutTimer.elapsed_time().count() > 30000) return 999.0; 
    }
    
    echoTimer.start();
    
    // Wait for echo pin to go LOW
    while (echo == 1) {
        // If it takes longer than 60ms, timeout.
        if (timeoutTimer.elapsed_time().count() > 60000) return 999.0;
    }
    
    echoTimer.stop();
    
    // Calculate and return the distance in cm
    return (echoTimer.elapsed_time().count() * 0.0343) / 2.0;
}

// The Background Worker Thread
void ultrasonicWorker() {
    while (true) {
       //printf("Thread Alive! Starting Ping...\r\n"); // PROOF OF LIFE
        // 1. Ping the Front Sensor
        double front = pingUltrasonic(frontTrigger, frontEcho);
        //if (front < 400.0) { // Filter out bad massive readings
            globalFrontDistance = front;
            printf("Front Dist: %d\r\n", int(globalFrontDistance));
        //}
        if (globalFrontDistance <= 20.0 && !objectDetected) {
            objectDetected = true;
            //printf("Object Detected\r\n");
        }
        else {
            objectDetected = false;
        }
        // Wait 30ms to let the soundwaves dissipate (prevents cross-talk)
        thread_sleep_for(30); 
    }
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

void avoidObstacleHard() {
    Timer orbitTimer;

    // STEP 1: The Evasion Turn
    // Turn right just enough so the robot clears the front of the object.
    // (You will need to tune this 400000us / 0.4s value to match your 90-degree turn time)
    cornerRight(dutyTurnRight);
    wait_us(400000); 

    // Stop briefly to let momentum settle before accelerating again
    fullStop();
    wait_us(100000);

    // STEP 2: Lock into the Fixed Arc
    // Set both wheels to drive forward, but make the left wheel slower.
    // This forces the robot to drive in a fixed left-handed circle.
    leftMotor.write(duty - 0.15f); // Slower inside wheel
    rightMotor.write(duty);        // Faster outside wheel
    
    leftForwardControl  = 1; rightForwardControl = 1;
    leftBackwardControl = 0; rightBackwardControl = 0;

    // STEP 3: The "Wait and Seek"
    orbitTimer.start();
    
    // The robot will continuously drive in its blind arc until the IR sensors
    // detect the line. We include a 5.0-second failsafe so it doesn't orbit forever.
    while (!lineDetected()) {
        if (orbitTimer.read() > 5.0f) {
            break; // Failsafe triggered: we missed the line, stop the robot
        }
        
        // We do absolutely nothing in this loop! 
        // We just let the CPU spin while the motors drive the arc.
        wait_us(10000); 
    }

    // STEP 4: Cleanup
    orbitTimer.stop();
    fullStop();
    
    // Pause briefly to stabilize before returning to the main line-following loop
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

    //printf("Calibrating... Keep sensor over a neutral background (white/gray).\r\n");

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
    dynamicMaxRed = ambientRed * 0.95;

    //printf("Calibration Done! New MinGap: %.2ld | New MaxRed: %.2ld\r\n", long(dynamicMinGap), long(dynamicMaxRed));

    return dynamicMaxRed;
}

void colorSensorWorker() {
    while (true) {
        // Read the color sensor
        bool detected = redDetected();
        
        // Update the global flag that the main loop is watching
        isStoppedForRed = detected;
        
        // Yield the CPU for 50 milliseconds. 
        // This is crucial! It tells the RTOS to pause this thread and give 
        // 100% of the processing power back to the motors for 50ms.
        thread_sleep_for(50); 
    }
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
    ultrasonicThread.start(ultrasonicWorker);
    colorThread.start(colorSensorWorker);
    
    // 4. Your incredibly fast, uninterrupted Main Loop
    while (true) {
        printf("Front Dist: %d\r\n", int(globalFrontDistance));
        printf("Side Dist: %d\r\n", int(globalSideDistance));

        // Check the flag updated by the background thread
        if (isStoppedForRed) {
            fullStop();
            continue; // Skip the IR logic while red is detected
        }
            
        /*cm and if 20 cm then 5882.35 us time to bounce back*/
        if (objectDetected) {
            avoidObstacleHard();
            continue;
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
            forward(0.1);

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
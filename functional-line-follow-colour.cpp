#undef __ARM_FP
#include "mbed.h"
#include "TCS3200.h"

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

DigitalOut red_led(LED_RED);
// Pin configuration: S0, S1, S2, S3, OUT
TCS3200 colourSensor(PTC1, PTC2, PTB3, PTB2, PTA13);
// ── Serial Ports ───────────────────────────────
static BufferedSerial pc_serial(USBTX, USBRX, 9600);
static BufferedSerial bt_serial(PTA2,  PTA1,  9600);

float avgRed = 0;
float avgGreen = 0;
float avgBlue = 0;
float ambientRed = 0;
float ambientGreen = 0;
float ambientBlue = 0;
float dynamicMinGap = 50.0;
float dynamicMaxRed = 75.0;

float period = 1.0/40000;
float duty = 0.5;
float dutyTurnRight = 0.8;
float dutyTurnLeft = 0.8;

Thread colorThread(osPriorityNormal, 1024);
volatile bool isStoppedForRed = false; 

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
        wait_us(35000);

}

void cornerRight(float dutyTurnLeft) {
    turnRight(dutyTurnLeft);
        wait_us(35000);

}

void fullStop() {
    leftMotor.write(0.8f);
    rightMotor.write(0.8f);

    leftForwardControl  = 1;
    leftBackwardControl = 1;
    rightForwardControl = 1;
    rightBackwardControl = 1;
}

bool redDetected() {
    float alpha = 0.9; 
    long rawRed = colourSensor.ReadRed();
    long rawGreen = colourSensor.ReadGreen();
    long rawBlue = colourSensor.ReadBlue();
    
    // Apply Filter to current reading
    avgRed = (rawRed * alpha) + (avgRed * (1.0 - alpha));
    avgGreen = (rawGreen * alpha) + (avgGreen * (1.0 - alpha));
    avgBlue = (rawBlue * alpha) + (avgBlue * (1.0 - alpha));
    
    float blueGap = avgBlue - avgRed;
    float greenGap = avgGreen - avgRed;
        
    if (blueGap > dynamicMinGap && greenGap > dynamicMinGap && avgRed < dynamicMaxRed) {
        red_led = 0; // Turn ON
        return true;
    } 
    else {
        red_led = 1; // Turn OFF

        // 2. THE PROPORTIONAL AMBIENT TRACKER
        // Check if the current light is within +/- 30% of our known ambient baseline.
        // This naturally ignores the black line and red marks (which are massive spikes),
        // but perfectly tracks shadows, clouds, and draining batteries.
        bool isWhiteTrack = (avgRed < ambientRed * 1.3) && (avgRed > ambientRed * 0.7) &&
                            (avgGreen < ambientGreen * 1.3) && (avgGreen > ambientGreen * 0.7) &&
                            (avgBlue < ambientBlue * 1.3) && (avgBlue > ambientBlue * 0.7);
        
        if (isWhiteTrack) {
            
            // Safe, slow update rate (5%). Tracks gradual changes without jitter.
            float ambientAlpha = 0.05; 
            
            ambientRed = (avgRed * ambientAlpha) + (ambientRed * (1.0 - ambientAlpha));
            ambientGreen = (avgGreen * ambientAlpha) + (ambientGreen * (1.0 - ambientAlpha));
            ambientBlue = (avgBlue * ambientAlpha) + (ambientBlue * (1.0 - ambientAlpha));
            
            // Recalculate natural ambient gaps based on the shifting light
            float ambientBlueGap = ambientBlue - ambientRed;
            float ambientGreenGap = ambientGreen - ambientRed;
            
            // Update the thresholds
            dynamicMinGap = ((ambientBlueGap + ambientGreenGap) / 2.0) + 15.0;
            
            // SAFETY FLOOR: Never let the required gap shrink below 20, 
            // even if the room gets incredibly bright and washes out the colors.
            if (dynamicMinGap < 20.0) {
                dynamicMinGap = 20.0; 
            }
            
            dynamicMaxRed = ambientRed * 0.90; 
        }

        return false;
    }
}

double baselineRed() {
    long totalRed = 0;
    long totalGreen = 0;
    long totalBlue = 0;
    int numSamples = 20;

    printf("Calibrating... Keep sensor over a neutral background (white/gray).\r\n");

    for (int i = 0; i < numSamples; i++) {
        totalRed += colourSensor.ReadRed();
        totalGreen += colourSensor.ReadGreen();
        totalBlue += colourSensor.ReadBlue();
        wait_us(10000); 
    }

    // Set the GLOBAL ambient variables
    ambientRed = (float)totalRed / numSamples;
    ambientGreen = (float)totalGreen / numSamples;
    ambientBlue = (float)totalBlue / numSamples;

    float ambientBlueGap = ambientBlue - ambientRed;
    float ambientGreenGap = ambientGreen - ambientRed;

    dynamicMinGap = ((ambientBlueGap + ambientGreenGap) / 2.0) + 10.0;
    dynamicMaxRed = ambientRed * 0.95;

    printf("Calibration Done! New MinGap: %.2ld | New MaxRed: %.2ld\r\n", long(dynamicMinGap), long(dynamicMaxRed));

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
    colourSensor.SetMode(TCS3200::SCALE_20);
    dynamicMaxRed = baselineRed();
    colorThread.start(colorSensorWorker);
    while (true) {
        printf("R: %d   G: %d   B: %d   Rth: %d Gap: %d\r\n", int(avgRed), int(avgGreen), int(avgBlue), int(dynamicMaxRed), int(dynamicMinGap));
        if (isStoppedForRed) {
            fullStop();
            continue; 
        }
        int leftValue = leftIR.read();
        int rightValue = rightIR.read();
        int leftTurnValue = leftTurnIR.read();
        int rightTurnValue = rightTurnIR.read();
        int middleValue = middleIR.read();
        //printf("Far Right:%d, Right: %d, Middle: %d, Left: %d, Far Left: %d\r\n", rightTurnValue, rightValue, middleValue, leftValue, leftTurnValue);
 
        // If 90 degree right turn is needed
        if ((leftValue == 1 && rightValue == 1 && rightTurnValue == 1 && middleValue == 1 && leftTurnValue == 0) || (leftTurnValue == 0 && 
        leftValue == 0 && middleValue == 1 && rightValue == 1 && rightTurnValue == 1 )) {
            //printf("Cornering Right\r\n");
            cornerRight(dutyTurnRight);
        }
        // If 90 degree left turn is needed
        else if ((leftValue == 1 && rightValue == 1 && rightTurnValue == 0 && middleValue == 1 && leftTurnValue == 1) || (leftTurnValue == 1 && 
        leftValue == 0 && middleValue == 1 && rightValue == 0 && rightTurnValue == 0 )) {
            //printf("Cornering Left\r\n");
            cornerLeft(dutyTurnLeft);
        }
        // If both sensors are on BLACK, move forward
        else if (leftValue == 1 && rightValue == 1 && middleValue == 1 && leftTurnValue == 0 && rightTurnValue == 0) {
            //printf("Forward\r\n");
            forward(duty - 0.1);
        }
        // Both sensors on WHITE, stop
        else if (leftValue == 1 && rightValue == 1 && middleValue == 1 && rightTurnValue == 1 && leftTurnValue == 1) {
            //printf("Stopping\r\n");
            stop();
        }   
        // Left sensor on black line, turn left
        else if ( (leftTurnValue == 0 && leftValue == 1 &&  middleValue == 1 && rightValue == 0  &&  rightTurnValue == 0 )|| (leftTurnValue == 1 && leftValue == 1)) {
            //printf("Turning Left\r\n");
            turnLeft(duty);
        }
        // Right sensor on black line, turn right
        else if ( (leftTurnValue == 0 && leftValue == 0 &&  middleValue == 1 && rightValue == 1  &&  rightTurnValue == 0 )|| (rightTurnValue == 1 && rightValue == 1)) {
            //printf("Turning Right\r\n");
            turnRight(duty);
        }
        // If only the far left is on line turn right to get back on line
        else if (leftTurnValue == 1 && leftValue == 0 && middleValue == 0 && rightValue == 0 && rightTurnValue == 0) {
            turnLeft(duty);
        }
        //If only the far right is on Line turn left to get back on line
        else if (leftTurnValue == 0 && leftValue == 0 && middleValue == 0 && rightValue == 0 && rightTurnValue == 1) {
            turnRight(duty);
        }
    }
}
#undef __ARM_FP
#include "mbed.h"
#include "TCS3200.h"

//Left Motor Pins
PwmOut leftMotor(PTA1);
DigitalOut leftForwardControl(PTA2);
DigitalOut leftBackwardControl(PTD4);
//Right Motor Pins
PwmOut rightMotor(PTA12);
DigitalOut rightForwardControl(PTA5);
DigitalOut rightBackwardControl(PTA4);

//IR Sensor Pins
DigitalIn leftIR(PTD5);
DigitalIn middleIR(PTD0);
DigitalIn rightIR(PTD2);
DigitalIn rightTurnIR(PTD3);
DigitalIn leftTurnIR(PTA13);

//Ultrasonic Sensor Pins
DigitalOut frontTrigger(PTD1);
DigitalIn frontEcho(PTE0);
DigitalOut sideTrigger(PTC6);
DigitalIn sideEcho(PTC7);

DigitalOut redLed(LED1);
DigitalOut greenLed(LED2);
DigitalOut blueLed(LED3);

DigitalInOut DHT22_PIN(PTC1);  // DHT22 Data pin

// ── Serial Ports ───────────────────────────────
static BufferedSerial pc_serial(USBTX, USBRX, 9600);
static BufferedSerial bt_serial(PTC9,  PTC8,  9600);

// Pin configuration: S0, S1, S2, S3, OUT
TCS3200 colorSensor(D2, D3, D4, D5, D6); 

// ── Temperature Thresholds (°C) ────────────────
#define TEMP_MIN        0.0f
#define TEMP_WARN       45.0f
#define TEMP_SHUTDOWN   100.0f

// ── Humidity Thresholds (%) ────────────────────
#define HUM_MIN         10.0f
#define HUM_WARN        75.0f
#define HUM_SHUTDOWN    85.0f

float period = 1.0/40000;
float duty = 0.6;
float dutyTurnRight = 0.8767;
float dutyTurnLeft = 0.6767;

FileHandle *mbed::mbed_override_console(int) {
    return &pc_serial;
}

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
            Echo_High_TimeFront = High_TimeS.elapsed_time().count();
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
    double objectDistanceSide = 0;
    while (getDistanceSide(objectDistanceSide) > 20) {
            turnRight(duty);
    }
    while(!lineDetected()) {
        objectDistanceSide = getDistanceSide(objectDistanceSide);
        if (objectDistanceSide > 20) {
            turnLeft(duty);
        }
        if (objectDistanceSide < 20) {
            turnRight(duty);
        }
        else {
            forward(duty);
        }
    }
}

// ── Rover States ───────────────────────────────
enum RoverState {
    ROVER_SAFE,
    ROVER_WARNING,
    ROVER_SHUTDOWN
};

// ═══════════════════════════════════════════════
// BT & Temp HELPER FUNCTIONS
// ═══════════════════════════════════════════════

void pcPrint(const char* msg) {
    pc_serial.write(msg, strlen(msg));
}

void btPrint(const char* msg) {
    bt_serial.write(msg, strlen(msg));
}

void printBoth(const char* msg) {
    pcPrint(msg);
    btPrint(msg);
}

// ═══════════════════════════════════════════════
// DHT22 READ FUNCTION
// ═══════════════════════════════════════════════

bool readDHT22(float &temperature, float &humidity) {
    uint8_t data[5] = {0};

    // --- Send START signal ---
    DHT22_PIN.output();
    DHT22_PIN = 0;
    wait_us(1100);      // Pull LOW 1.1ms
    DHT22_PIN = 1;
    wait_us(30);        // Pull HIGH 30us
    DHT22_PIN.input();

    // --- Wait for DHT22 response ---
    int timeout = 0;
    while (DHT22_PIN == 1) { wait_us(1); if (++timeout > 200) return false; }
    timeout = 0;
    while (DHT22_PIN == 0) { wait_us(1); if (++timeout > 200) return false; }
    timeout = 0;
    while (DHT22_PIN == 1) { wait_us(1); if (++timeout > 200) return false; }

    // --- Read 40 bits (5 bytes) ---
    for (int i = 0; i < 40; i++) {
        // Wait for LOW to HIGH transition
        timeout = 0;
        while (DHT22_PIN == 0) { wait_us(1); if (++timeout > 200) return false; }

        // Measure HIGH pulse width
        wait_us(40);

        // If still HIGH after 40us = bit 1, else bit 0
        data[i / 8] <<= 1;
        if (DHT22_PIN == 1) {
            data[i / 8] |= 1;
            timeout = 0;
            while (DHT22_PIN == 1) { wait_us(1); if (++timeout > 200) return false; }
        }
    }

    // --- Verify checksum ---
    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        return false;
    }

    // --- Convert raw bytes to float ---
    humidity    = ((data[0] << 8) | data[1]) / 10.0f;
    temperature = (((data[2] & 0x7F) << 8) | data[3]) / 10.0f;
    if (data[2] & 0x80) temperature = -temperature; // Negative temp

    return true;
}

// ═══════════════════════════════════════════════
// ROVER STATE CHECK
// ═══════════════════════════════════════════════

RoverState checkConditions(float temp, float hum) {

    // Shutdown conditions
    /*if (temp >= TEMP_SHUTDOWN || temp <= TEMP_MIN ||
        hum  >= HUM_SHUTDOWN  || hum  <= HUM_MIN) {
        return ROVER_SHUTDOWN;
    }*/

    // Warning conditions
    if (temp >= TEMP_WARN || hum >= HUM_WARN) {
        return ROVER_WARNING;
    }

    return ROVER_SAFE;
}

// ═══════════════════════════════════════════════
// APPLY ROVER STATE
// ═══════════════════════════════════════════════

void applyRoverState(RoverState state, float temp, float hum) {

    switch (state) {

        case ROVER_SAFE:
            greenLed    = 0;    // Green ON  (active low)
            redLed      = 1;    // Red OFF
            blueLed     = 1;    // Blue OFF
            printBoth("STATUS: SAFE - Rover Running\r\n");
            break;

        case ROVER_WARNING:
            greenLed    = 1;    // Green OFF
            redLed      = 1;    // Red OFF
            blueLed     = 0;    // Blue ON = warning
            printBoth("STATUS: WARNING - High Temp/Humidity!\r\n");
            break;

        case ROVER_SHUTDOWN:
            fullStop();
            greenLed    = 1;    // Green OFF
            redLed      = 0;    // Red ON = danger
            blueLed     = 1;    // Blue OFF
            printBoth("STATUS: SHUTDOWN - Unsafe Conditions!\r\n");

            // Print exact shutdown reason
            if (temp >= TEMP_SHUTDOWN) {
                printBoth("REASON: Temperature too HIGH!\r\n");
            } else if (temp <= TEMP_MIN) {
                printBoth("REASON: Temperature too LOW!\r\n");
            } else if (hum >= HUM_SHUTDOWN) {
                printBoth("REASON: Humidity too HIGH!\r\n");
            } else if (hum <= HUM_MIN) {
                printBoth("REASON: Humidity too LOW!\r\n");
            }
            break;
    }
}

void humidityControl() {
    float temp = 0.0f;
    float hum  = 0.0f;

    // ── Read DHT22 ─────────────────────────
    if (readDHT22(temp, hum)) {

        // ── Format values manually ──────────
        int tInt = (int)temp;
        int tDec = (int)((temp - tInt) * 10);
        int hInt = (int)hum;
        int hDec = (int)((hum  - hInt) * 10);

        // ── Print readings ──────────────────
        char buf[64];

        sprintf(buf, "Temp: %d.%d C | Humidity: %d.%d%%\r\n",
                tInt, tDec, hInt, hDec);
        pcPrint(buf);

        sprintf(buf, "T: %d.%d C  H: %d.%d%%\r\n",
                tInt, tDec, hInt, hDec);
        btPrint(buf);

        // ── Check conditions & act ──────────
        RoverState state = checkConditions(temp, hum);
        applyRoverState(state, temp, hum);

        // ── Print separator ─────────────────
        printBoth("------------------------------\r\n");

    } else {
        // Sensor read failed → shutdown for safety
        fullStop();
        redLed      = 0;
        printBoth("ERROR: DHT22 Read Failed!\r\n");
        printBoth("SAFETY: Motors Disabled!\r\n");
        printBoth("Check wiring & 10k pullup!\r\n");
        printBoth("------------------------------\r\n");
    }
}

bool redDetected() {
    long redValue = colorSensor.ReadRed();
    if (redValue < 50) {
        return true;
    }
    return false;
}

int main()
{
    stop();
    leftMotor.period(period);
    rightMotor.period(period);
    double Object_Distance_Front = 0;
    colorSensor.SetMode(TCS3200::SCALE_20); 

    while (true) {
        while (redDetected()) { // If colour sensor detects red
            fullStop();
        }

        Object_Distance_Front = ultrasonicHandlerFront(Object_Distance_Front);
        
        /*cm and if 20 cm then 5882.35 us time to bounce back*/
        if (Object_Distance_Front <= 20.0) {
            avoidObstacle();
        }
        int leftValue = leftIR.read();
        int rightValue = rightIR.read();
        int leftTurnValue = leftTurnIR.read();
        int rightTurnValue = rightTurnIR.read();
        int middleValue = middleIR.read();

        // If 90 degree right turn is needed
        if ((leftValue == 1 && rightValue == 1 && rightTurnValue == 1 && middleValue == 1 && leftTurnValue == 0) || (leftTurnValue == 0 && leftValue == 0 && middleValue == 1 && rightValue == 1 && rightTurnValue == 1 )) {
            printBoth("Initiating 90 degree turn Right");
            cornerRight(dutyTurnRight);
        }
        // If 90 degree left turn is needed
        if ((leftValue == 1 && rightValue == 1 && rightTurnValue == 0 && middleValue == 1 && leftTurnValue == 1) || (leftTurnValue == 1 && leftValue == 0 && middleValue == 1 && rightValue == 0 && rightTurnValue == 0 )) {
            printBoth("Initiating 90 degree turn Left");
            cornerLeft(dutyTurnLeft);
        }
        // If both sensors are on BLACK, move forward
        else if (leftValue == 1 && rightValue == 1 && middleValue == 1 && leftTurnValue == 0 && rightTurnValue == 0) {
            forward(duty - 0.3);
        }
        // Left sensor on black line, turn left
        else if ( (leftTurnValue == 0 && leftValue == 0 &&  middleValue == 1 && rightValue == 1  &&  rightTurnValue == 0 )|| (leftTurnValue == 1 && leftValue == 1)) {
            printBoth("Initiating Left Turn");
            turnLeft(duty);
        }
        // Right sensor on black line, turn right
        else if ( (leftTurnValue == 0 && leftValue == 0 &&  middleValue == 1 && rightValue == 1  &&  rightTurnValue == 0 )|| (rightTurnValue == 1 && rightValue == 1)) {
            printBoth("Initiating Right Turn");
            turnRight(duty);
        }
             // Both sensors on WHITE, stop
        else if (leftValue == 1 && rightValue == 1 && middleValue == 1 && rightTurnValue == 1 && leftTurnValue == 1) {
            printBoth("Braking");
            fullStop();
        }
        //Call the humidity control method
        humidityControl();
    }
}
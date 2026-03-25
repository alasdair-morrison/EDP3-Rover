#include "mbed.h"

// ── Pin Definitions ────────────────────────────
#define DHT22_PIN       PTD4    // DHT22 Data pin

// ── Temperature Thresholds (°C) ────────────────
#define TEMP_MIN        0.0f
#define TEMP_WARN       45.0f
#define TEMP_SHUTDOWN   100.0f

// ── Humidity Thresholds (%) ────────────────────
#define HUM_MIN         10.0f
#define HUM_WARN        75.0f
#define HUM_SHUTDOWN    85.0f

// ── Serial Ports ───────────────────────────────
static BufferedSerial pc_serial(USBTX, USBRX, 9600);
static BufferedSerial bt_serial(PTA2,  PTA1,  9600);

// ── Override console for printf ────────────────
FileHandle *mbed::mbed_override_console(int) {
    return &pc_serial;
}

// ── LEDs ───────────────────────────────────────
DigitalOut redLed(LED1);
DigitalOut greenLed(LED2);
DigitalOut blueLed(LED3);

// ── Motor Enable Pin ───────────────────────────
//DigitalOut motorEnable(PTD5);   // Change pin if needed

// ── Rover States ───────────────────────────────
enum RoverState {
    ROVER_SAFE,
    ROVER_WARNING,
    ROVER_SHUTDOWN
};

// ═══════════════════════════════════════════════
// HELPER FUNCTIONS
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
    DigitalInOut pin(DHT22_PIN);
    pin.output();
    pin = 0;
    wait_us(1100);      // Pull LOW 1.1ms
    pin = 1;
    wait_us(30);        // Pull HIGH 30us
    pin.input();

    // --- Wait for DHT22 response ---
    int timeout = 0;
    while (pin == 1) { wait_us(1); if (++timeout > 200) return false; }
    timeout = 0;
    while (pin == 0) { wait_us(1); if (++timeout > 200) return false; }
    timeout = 0;
    while (pin == 1) { wait_us(1); if (++timeout > 200) return false; }

    // --- Read 40 bits (5 bytes) ---
    for (int i = 0; i < 40; i++) {
        // Wait for LOW to HIGH transition
        timeout = 0;
        while (pin == 0) { wait_us(1); if (++timeout > 200) return false; }

        // Measure HIGH pulse width
        wait_us(40);

        // If still HIGH after 40us = bit 1, else bit 0
        data[i / 8] <<= 1;
        if (pin == 1) {
            data[i / 8] |= 1;
            timeout = 0;
            while (pin == 1) { wait_us(1); if (++timeout > 200) return false; }
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
            //motorEnable = 1;    // Motors ON
            greenLed    = 0;    // Green ON  (active low)
            redLed      = 1;    // Red OFF
            blueLed     = 1;    // Blue OFF
            printBoth("STATUS: SAFE - Rover Running\r\n");
            break;

        case ROVER_WARNING:
            //motorEnable = 1;    // Still running
            greenLed    = 1;    // Green OFF
            redLed      = 1;    // Red OFF
            blueLed     = 0;    // Blue ON = warning
            printBoth("STATUS: WARNING - High Temp/Humidity!\r\n");
            break;

        case ROVER_SHUTDOWN:
            //motorEnable = 0;    // Motors OFF
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

// ═══════════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════════

int main() {

    // Startup
    //motorEnable = 0;    // Motors off on boot
    redLed      = 1;    // All LEDs off (active low)
    greenLed    = 1;
    blueLed     = 1;

    printBoth("==============================\r\n");
    printBoth("   ROVER SYSTEM BOOTING...    \r\n");
    printBoth("==============================\r\n");

    ThisThread::sleep_for(2000ms); // DHT22 warmup

    printBoth("System Ready!\r\n");

    while (1) {

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
            //motorEnable = 0;
            redLed      = 0;
            printBoth("ERROR: DHT22 Read Failed!\r\n");
            printBoth("SAFETY: Motors Disabled!\r\n");
            printBoth("Check wiring & 10k pullup!\r\n");
            printBoth("------------------------------\r\n");
        }

        ThisThread::sleep_for(2500ms);
    }
}
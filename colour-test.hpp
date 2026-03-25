#undef __ARM_FP
#include "mbed.h"
#include "TCS3200.h"
#include <list>
#include <numeric>

DigitalOut red_led(LED_RED);
// Pin configuration: S0, S1, S2, S3, OUT
TCS3200 colourSensor(PTC1, PTC2, PTB3, PTB2, PTA13);
static BufferedSerial pc(USBTX, USBRX, 9600);
float avgRed = 0;
float avgGreen = 0;
float avgBlue = 0;

void readSmoothedColors() {
    // Tunable "Smoothness" (0.1 = Very Smooth/Slow, 0.9 = Fast/Jittery)
    float alpha = 0.6; 
    
    long rawRed = colourSensor.ReadRed();
    long rawGreen = colourSensor.ReadGreen();
    long rawBlue = colourSensor.ReadBlue();
    
    // Apply Filter
    avgRed = (rawRed * alpha) + (avgRed * (1.0 - alpha));
    avgGreen = (rawGreen * alpha) + (avgGreen * (1.0 - alpha));
    avgBlue = (rawBlue * alpha) + (avgBlue * (1.0 - alpha));
    
}

int main() {
    colourSensor.SetMode(TCS3200::SCALE_20);
    printf("Sensor Initialized. Starting Loop...\r\n");
    float minGap = 50.0;
    while(1) {
        readSmoothedColors();
        // 4. Print to Serial Monitor
        // %ld is for "long" integers. \r\n creates a new line.
        printf("R: %d   G: %d   B: %d\r\n", int(avgRed), int(avgGreen), int(avgBlue));
        float blueGap = avgBlue - avgRed;
        float greenGap = avgGreen - avgRed;
        
        // 5. The "Contrast" Check
        // "Is Blue 50 units weaker than Red? AND Is Green 50 units weaker?"
        if (blueGap > minGap && greenGap > minGap && avgRed < 200) {
            red_led = 0; // Turn ON
            printf("Red Detected\r\n");
        } 
        else {
            red_led = 1; // Turn OFF
            printf("No Red Detected\r\n");
        }
        
        wait_us(100000); // Run faster (10Hz) so the filter can work
    }
}
#undef __ARM_FP
#include "mbed.h"
#include "RoverControl.hpp"

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
int threshold[SensorCount];

float Kp = 0;
float Ki = 0;
float Kd = 0;

uint8_t multiP = 1;
uint8_t multiI  = 1;
uint8_t multiD = 1;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
float Pvalue;
float Ivalue;
float Dvalue;

int val, cnt = 0, v[3];

uint16_t position;
int P, D, I, previousError, PIDvalue, curError;
int lsp, rsp;
int lfspeed = 230;

void sensorRead(int error) {
    P = error;
    I = I + error;
    D = error - previousError;
    
    Pvalue = (Kp/pow(10,multiP))*P;
    Ivalue = (Ki/pow(10,multiI))*I;
    Dvalue = (Kd/pow(10,multiD))*D; 

    float PIDvalue = Pvalue + Ivalue + Dvalue;
    previousError = error;

    lsp = lfspeed - PIDvalue;
    rsp = lfspeed + PIDvalue;

    if (lsp > 255) {
      lsp = 255;
    }
    if (lsp < -255) {
      lsp = -255;
    }
    if (rsp > 255) {
      rsp = 255;
    }
    if (rsp < -255) {
      rsp = -255;
    }
}

/*int main()
{
    stop();
    float period = 1.0/40000;
    float duty = 0.6;
    int wait_delay = 5000;
    leftMotor.period(period);
    rightMotor.period(period);

    while (true) {
        int leftValue = leftIR.read();
        int rightValue = rightIR.read();
        int leftTurnValue = leftTurnIR.read();
        int rightTurnValue = rightTurnIR.read();
        int middleValue = middleIR.read();

        if (leftValue == 0 && rightValue == 0 && middleValue == 0 && leftTurnValue == 0 && rightTurnValue == 0) {
            stop();
            //wait_us(wait_delay);
        }
        else if (leftValue == 1 && rightValue == 1 && middleValue == 1 && leftTurnValue == 1 && rightTurnValue == 1) {
            stop();
            //wait_us(wait_delay);
        }
        // If 90 degree right turn is needed
        else if (rightTurnValue == 1 && rightValue == 1 && middleValue == 1 && leftValue == 1) {
            cornerRight(duty);
            //wait_us(wait_delay);
        }
        // If 90 degree left turn is needed
        else if (leftTurnValue == 1 && leftValue == 1 && middleValue == 1 && rightValue == 1) {
            cornerLeft(duty);
            //wait_us(wait_delay);
        }
        // If both sensors are on line, move forward
        else if (leftValue == 1 && rightValue == 1 && middleValue == 1) {
            forward(duty);
            //wait_us(wait_delay);
        }
        // Left sensor on black line, turn left
        else if (leftValue == 0 && rightValue == 1 && middleValue == 1) {
            turnLeft(duty);
            //wait_us(wait_delay);
        }
        // Right sensor on black line, turn right
        else if (leftValue == 1 && rightValue == 0 && middleValue == 1) {
            turnRight(duty);
            //wait_us(wait_delay);
        }
    }
}*/

int main() {

}
#include "mbed.h"

DigitalIn Echo(PTA13);
DigitalOut Trigger(PTD2);
DigitalOut LED(LED1);
int FSM = 0;
Timer High_Time;
Timer Emergency_Timer;
double Echo_High_Time = 0.0;
double Object_Distance = 0.0;

int main(){

    // MUST SET TRIGGER TO HIGH FOR 10US THEN LOW, IF ECHO COMES BACK LONG THEN OBJECT
    while (true) {
        
        if (FSM == 0 && Echo == 0) {
            Trigger = 0;
            wait_us(1);   // Just a clean transition start
            Trigger = 1;
            wait_us(10);  // Set Trigger to High for 10 us
            Trigger = 0;

            Emergency_Timer.reset();     // we neeed thhis in case trigger goes of but echo does not due to a loose connection or any reason
            Emergency_Timer.start();

            FSM = 1;   // Finite-State Machine is on mode 1
        }

        if (FSM == 1 && Echo == 1) {
            High_Time.reset();
            High_Time.start();

            Emergency_Timer.stop();     // end the timer since if we get to this point we cant be stuck anymore

            FSM = 2;   // Finite-State Machine is on mode 2
        }

        if (FSM == 2 && Echo == 0) {
            High_Time.stop();
            FSM = 3;   // Finite-State Machine is on mode 2
        }    

        if (FSM == 3 && Echo == 0) {
            // Calculate time in us
            Echo_High_Time = High_Time.elapsed_time().count();
            // CONVERTED TO cm/us form m/s from datasheet
            Object_Distance = (Echo_High_Time * (0.034/ 2.0));
            FSM = 0;   // Finite-State Machine is on mode 2
        }    

        float Emergency_Time = Emergency_Timer.elapsed_time().count();

        /*cm and if 20 cm then 5882.35 us time to bounce back*/
        if (Object_Distance <= 20.0){LED = 1;}
        else {LED = 0;}

        if ((FSM == 1 || FSM == 2) && Emergency_Time > 60000.0){
            Emergency_Timer.stop();     // we neeed this in case trigger goes of but echo does not due to a loose connection or any reason
            Emergency_Timer.reset();
            FSM = 0; 
        } // Prevent being stuck if wire is loose and echo nevcer goes high
    }
}

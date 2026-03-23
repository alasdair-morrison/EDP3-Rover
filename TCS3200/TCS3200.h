/* TCS3200 Library v1.0
 * Copyright (c) 2016 Grant Phillips
 * grant.phillips@nmmu.ac.za
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
 
#ifndef TCS3200_H
#define TCS3200_H
 
#include "mbed.h"

/** Class library for a TCS3200 Color Sensor Module.
 *
 * Example:
 * @code
 * #include "mbed.h"
 * #include "TCS3200.h"
 *
 * TCS3200 color(PD_8, PD_9, PD_10, PD_11, PA_1);  //Create a TCS3200 object 
 * //            S0    S1    S2     S3     OUT
 * 
 * int main() {
 *     long red, green, blue, clear;
 *    
 *     //Set the scaling factor to 100%
 *     color.SetMode(TCS3200::SCALE_100);        
 *
 *     while(1){
 *         //Read the HIGH pulse width in nS for each color.
 *         //The lower the value, the more of that color is detected 
 *         red = color.ReadRed();
 *         green = color.ReadGreen();
 *         blue = color.ReadBlue();
 *         clear = color.ReadClear();
 *
 *         printf("RED: %10d     GREEN: %10d     BLUE: %10d     CLEAR: %10d     ", red, green, blue, clear);
 *            
 *         wait(0.1);
 *     }
 * }
 * @endcode
 */
 
class TCS3200 {
  public:
    enum TCS3200Mode {
        POWERDOWN = 0,
        SCALE_2 = 1,
        SCALE_20 = 2,
        SCALE_100 = 3,
    };
    
    /** Create a TCS3200 object connected to the specified pins.
    * @param S0 Frequency scaling output pin S0
    * @param S1 Frequency scaling output pin S1
    * @param S2 Photo diode selection output pin S2
    * @param S3 Photo diode selection output pin S3
    * @param OUT Frequency input pin
    */
    TCS3200(PinName S0, PinName S1, PinName S2, PinName S3, PinName OUT);
    
    /** Reads the output signal's HIGH pulse for RED.
    * @param 
    *     None
    * @return 
    *     Duration as nanoseconds (ns).
    */
    long ReadRed();
    
    /** Reads the output signal's HIGH pulse for GREEN.
    * @param 
    *     None
    * @return 
    *     Duration as nanoseconds (ns).
    */
    long ReadGreen();
    
    /** Reads the output signal's HIGH pulse for BLUE.
    * @param 
    *     None
    * @return 
    *     Duration as nanoseconds (ns).
    */
    long ReadBlue();
    
    /** Reads the output signal's HIGH pulse for CLEAR.
    * @param 
    *     None
    * @return 
    *     Duration as nanoseconds (ns).
    */
    long ReadClear();
    
    /** Sets the mode of operation. 
    * @param 
    *     mode POWERDOWN, SCALE_2 (2% scaling), SCALE_20 (20% scaling), SCALE_100 (100% scaling).
    * @return 
    *     None
    */
    void SetMode(TCS3200Mode mode);
    
  private:
    DigitalOut mS0;
    DigitalOut mS1;
    DigitalOut mS2;
    DigitalOut mS3;
    InterruptIn signal;
    Timer timer;
    long pulsewidth;
    void HighTrigger();
    void LowTrigger();
};
 
#endif
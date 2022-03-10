#ifndef TFTSHOW_H_
#define TFTSHOW_H_
#include <Arduino.h>
#include "m_servo.h"
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
//extern TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
class TFTshow:public m_servo 
{
    
    public:
    TFTshow():m_servo(0){}
    void TFTINIT();
    void BasicParametersShow();
    void EndTrackShow(float *param);
    void ParaCurveShow();
    private:

};




#endif
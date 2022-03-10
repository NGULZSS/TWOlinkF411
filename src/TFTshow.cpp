#include "TFTshow.h"
TFT_eSPI tft = TFT_eSPI(); // Invoke library, pins defined in User_Setup.h

    void TFTshow::TFTINIT()
    {
        tft.init();
        tft.setRotation(0);
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
    }
    void TFTshow::BasicParametersShow()
    {
        // tft.drawCentreString("Basic Parameter",60,0,2);
        // tft.drawCentreString("ID",30,15,2);
        // tft.drawCentreString("ActAngle",30,30,2);
        // tft.drawCentreString("ExpAngle",30,45,2);
        // tft.drawCentreString("Mode",30,60,2);
        // tft.drawCentreString("Cur_Voltage",40,75,2);
        // tft.drawCentreString("Cur_Current",40,90,2);
        // tft.drawCentreString("Temp",30,105,2);
    }
    void TFTshow::EndTrackShow(float *param)
    {
        tft.drawNumber(param[0],2,80,15);
        tft.drawFloat(param[1],2,80,30);
        // tft.drawFloat(param[2],2,80,45);
        // tft.drawFloat(1,2,80,60);
        // tft.drawFloat(param[5],2,80,75);
        // tft.drawFloat(param[6],2,80,90);
        // tft.drawFloat(param[8],2,80,105);
    }
    void TFTshow::ParaCurveShow()
    {

    }



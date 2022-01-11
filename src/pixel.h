/**
 * @file pixel.c
 * @author Maciej Kurcius
 * @brief 
 * @version 0.1
 * @date 2022-02-11
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef PIXEL_H
#define PIXEL_H

#include <Arduino.h>
#include <SPI.h>
#include "hardware_cfg.h"

class PixelLedClass {
    public:
        PixelLedClass();
        PixelLedClass(uint8_t StripLength_, SPIClass* SPI_);
        ~PixelLedClass();
        void Init();
        void SendStartFrame();
        void SendStopFrame();
        void SendBuffersData();
        uint8_t GetStripLength(void);
        void SetStripBrightness(uint8_t Brightness_);
        void SetStripColour(uint8_t Red_, uint8_t Green_, uint8_t Blue_);
        void SetStripColour(uint8_t Red_, uint8_t Green_, uint8_t Blue_, uint8_t Brightness_);
        uint8_t SetNthLed(uint8_t Led_, uint8_t Red_, uint8_t Green_, uint8_t Blue_);
        uint8_t SetNthLed(uint8_t Led_, uint8_t Red_, uint8_t Green_, uint8_t Blue_, uint8_t Brightness_);
        void SetStrip(uint8_t* Red, uint8_t* Green, uint8_t* Blue, uint8_t* Brightness);
        void SetNthBrightness(uint8_t Led_, uint8_t Brightness);
        void SetGlobalBrighness(uint8_t Brightness);
        void TurnOffAllLeds();
    private:
        uint8_t StripLength;
        uint8_t* Red;
        uint8_t* Green;
        uint8_t* Blue;
        uint8_t* Brightness;
        SPIClass* PixelSpi;
        SPISettings* SpiSettings;
    protected:
    ;
};


#endif
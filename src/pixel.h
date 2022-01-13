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
#include <map>




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
        bool SetNthLed(uint8_t Led_, uint8_t Red_, uint8_t Green_, uint8_t Blue_);
        bool SetNthLed(uint8_t Led_, uint8_t Red_, uint8_t Green_, uint8_t Blue_, uint8_t Brightness_);
        void SetNthBrightness(uint8_t Led_, uint8_t Brightness_);
        void TurnOffAllLeds();
        void PixelStripMapInit();
        bool PixelStripMapRemap(uint8_t MapIndex_, uint8_t LedIndex_);
        bool PixelStripMapSwap(uint8_t LedIndex1_, uint8_t LedIndex2_);
    private:
        uint8_t StripLength;
        uint8_t* Red;
        uint8_t* Green;
        uint8_t* Blue;
        uint8_t* Brightness;
        SPIClass* PixelSpi;
        SPISettings* SpiSettings;
        std::map<uint8_t, uint8_t>PixelStripMap;
    protected:
    ;
};

class PixelStripSubsetClass {
    public:
    PixelStripSubsetClass();
    PixelStripSubsetClass(PixelLedClass *PixelStrip_);
    PixelStripSubsetClass(PixelLedClass *PixelStrip_, uint8_t FirstLed_, uint8_t LastLed_);
    ~PixelStripSubsetClass();
    bool SetSubset(uint8_t FirstLed_, uint8_t LastLed_);
    bool CheckErr(void);
    void StripMapFlip(void);
    void SetColour(uint8_t Red_, uint8_t Green_, uint8_t Blue_);
    void SetColour(uint8_t Red_, uint8_t Green_, uint8_t Blue_, uint8_t Brightness_);
    void SetBrightness(uint8_t Brightness_);
    void SetNthLedColour(uint8_t Red_, uint8_t Green_, uint8_t Blue_);
    void SetNthLedColour(uint8_t Red_, uint8_t Green_, uint8_t Blue_, uint8_t Brightness_);
    void SetNthLedBrightness(uint8_t Brightness_);
    private:
    bool ErrorFlag = false;
    PixelLedClass* PixelStrip;
    std::map<uint8_t, uint8_t>SubsetStripMap;
    uint8_t Length = 0;
};


#endif
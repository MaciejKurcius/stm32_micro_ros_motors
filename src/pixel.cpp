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

#include "pixel.h"

PixelLedClass::PixelLedClass(){
    ;
}

PixelLedClass::PixelLedClass(uint8_t StripLength_, SPIClass* Spi_){
    PixelSpi = Spi_;
    StripLength = StripLength_;
    Red = new uint8_t[StripLength_];
    Green = new uint8_t[StripLength_];
    Blue = new uint8_t[StripLength_];
    Brightness = new uint8_t [StripLength_];
    SpiSettings = new SPISettings(40000, LSBFIRST, SPI_MODE3, SPI_TRANSMITONLY);
    PixelSpi->beginTransaction(CS_PIN_CONTROLLED_BY_USER, *SpiSettings);
}

PixelLedClass::~PixelLedClass(){
    ;
}

void PixelLedClass::Init(){
    PixelSpi->beginTransaction(CS_PIN_CONTROLLED_BY_USER, *SpiSettings);
}

void PixelLedClass::SendStartFrame(){
    for(int i = 0; i < 4; i++){
        this->PixelSpi->transfer(CS_PIN_CONTROLLED_BY_USER, 0x00);
    }
}

void PixelLedClass::SendStopFrame(){
    for(int i = 0; i < 4; i++){
        this->PixelSpi->transfer(CS_PIN_CONTROLLED_BY_USER, 0xFF);
    }
}

void PixelLedClass::SendBuffersData(){
    this->SendStartFrame();
    for(int i = 0; i < StripLength; i++){
        this->PixelSpi->transfer(CS_PIN_CONTROLLED_BY_USER, (Brightness[i] | 0xE0));
        this->PixelSpi->transfer(CS_PIN_CONTROLLED_BY_USER, Blue[i]);
        this->PixelSpi->transfer(CS_PIN_CONTROLLED_BY_USER, Green[i]);
        this->PixelSpi->transfer(CS_PIN_CONTROLLED_BY_USER, Red[i]);
    }
    this->SendStopFrame();
}

uint8_t PixelLedClass::GetStripLength(void){
    return StripLength;
}

void PixelLedClass::SetStripBrightness(uint8_t Brightness_){
    for(int i = 0; i < StripLength; i++){
        Brightness[i] = Brightness_;
    }
    this->SendBuffersData();
}

void PixelLedClass::SetStripColour(uint8_t Red_, uint8_t Green_, uint8_t Blue_){
    for(int i = 0; i < StripLength; i++){
        Red[i] = Red_;
        Green[i] = Green_;
        Blue[i] = Blue_;
    }
    this->SendBuffersData();
}

void PixelLedClass::SetStripColour(uint8_t Red_, uint8_t Green_, uint8_t Blue_, uint8_t Brightness_){
    for(int i = 0; i < StripLength; i++){
        Red[i] = Red_;
        Green[i] = Green_;
        Blue[i] = Blue_;
        Brightness[i] = Brightness_;
    }
    this->SendBuffersData();
}

uint8_t PixelLedClass::SetNthLed(uint8_t Led_, uint8_t Red_, uint8_t Green_, uint8_t Blue_){
    if(Led_ >= StripLength)
        return 1;
    Red[Led_] = Red_;
    Green[Led_] = Green_;
    Blue[Led_] = Blue_;
    this->SendBuffersData();
    return 0;
}

uint8_t PixelLedClass::SetNthLed(uint8_t Led_, uint8_t Red_, uint8_t Green_, uint8_t Blue_, uint8_t Brightness_){
    if(Led_ >= StripLength)
        return 1;
    Red[Led_] = Red_;
    Green[Led_] = Green_;
    Blue[Led_] = Blue_;
    Brightness[Led_] = Brightness_;
    this->SendBuffersData();
    return 0;
}
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
    SpiSettings = new SPISettings(PIXEL_SPI_SPEED, LSBFIRST, SPI_MODE3, SPI_TRANSMITONLY);
    PixelSpi->beginTransaction(CS_PIN_CONTROLLED_BY_USER, *SpiSettings);
    PixelStripMapInit();
}

PixelLedClass::~PixelLedClass(){
    ;
}

void PixelLedClass::Init(){
    PixelSpi->beginTransaction(CS_PIN_CONTROLLED_BY_USER, *SpiSettings);
}

void PixelLedClass::SendStartFrame(){
    for(int i = 0; i < 4; i++)
        this->PixelSpi->transfer(CS_PIN_CONTROLLED_BY_USER, 0x00);
}

void PixelLedClass::SendStopFrame(){
    for(int i = 0; i < 4; i++)
        this->PixelSpi->transfer(CS_PIN_CONTROLLED_BY_USER, 0xFF);
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

bool PixelLedClass::SetNthLed(uint8_t Led_, uint8_t Red_, uint8_t Green_, uint8_t Blue_){
    if(Led_ >= StripLength)
        return true;
    Red[PixelStripMap[Led_]] = Red_;
    Green[PixelStripMap[Led_]] = Green_;
    Blue[PixelStripMap[Led_]] = Blue_;
    this->SendBuffersData();
    return false;
}

bool PixelLedClass::SetNthLed(uint8_t Led_, uint8_t Red_, uint8_t Green_, uint8_t Blue_, uint8_t Brightness_){
    if(Led_ >= StripLength)
        return true;
    Red[PixelStripMap[Led_]] = Red_;
    Green[PixelStripMap[Led_]] = Green_;
    Blue[PixelStripMap[Led_]] = Blue_;
    Brightness[PixelStripMap[Led_]] = Brightness_;
    this->SendBuffersData();
    return false;
}

void PixelLedClass::SetNthBrightness(uint8_t Led_, uint8_t Brightness_){
    Brightness[PixelStripMap[Led_]] = Brightness_;
    this->SendBuffersData();
}

void PixelLedClass::PixelStripMapInit(void){
    for(int i = 0; i < StripLength; i++)
        PixelStripMap[i] = i;
}

bool PixelLedClass::PixelStripMapRemap(uint8_t MapIndex_, uint8_t LedIndex_){
    if(MapIndex_ >= StripLength || LedIndex_ >= StripLength)
        return true;
    PixelStripMap[MapIndex_] = LedIndex_;
    return false;
}

bool PixelLedClass::PixelStripMapSwap(uint8_t LedIndex1_, uint8_t LedIndex2_){
    if(LedIndex1_ >= StripLength || LedIndex2_ >= StripLength)
        return true;
    PixelStripMap[LedIndex1_] = LedIndex2_;
    PixelStripMap[LedIndex2_] = LedIndex1_;
    return false;
}

PixelStripSubsetClass::PixelStripSubsetClass(){
    ;
}

PixelStripSubsetClass::PixelStripSubsetClass(PixelLedClass *PixelStrip_){
    PixelStrip = PixelStrip_;
}

PixelStripSubsetClass::PixelStripSubsetClass(PixelLedClass *PixelStrip_, uint8_t FirstLed_, uint8_t LastLed_){
    PixelStrip = PixelStrip_;
    if(FirstLed_ >= PixelStrip->GetStripLength() || LastLed_ >= PixelStrip->GetStripLength()){
        ErrorFlag = true;
        return;
    }
    Length = abs(LastLed_ - FirstLed_)+1;
    for(int i = 0; i < Length; i++){
        if(FirstLed_ <= LastLed_)
            SubsetStripMap[i] = FirstLed_ + i;
        else
            SubsetStripMap[i] = FirstLed_ - i;
    }
}

PixelStripSubsetClass::~PixelStripSubsetClass(){
    ;
}

bool PixelStripSubsetClass::SetSubset(uint8_t FirstLed_, uint8_t LastLed_){
    return false;
}

bool PixelStripSubsetClass::CheckErr(void){
    return ErrorFlag;
}


void PixelStripSubsetClass::StripMapFlip(void){
    std::map<uint8_t, uint8_t> temp_map;
    for(int i = 0; i < Length; i++)
        temp_map[i] = SubsetStripMap[(Length-1) - i];
}


void PixelStripSubsetClass::SetColour(uint8_t Red_, uint8_t Green_, uint8_t Blue_){
    uint8_t temp=0;
    for(int i=0; i < Length; i++){
        temp = SubsetStripMap[i];
        this->PixelStrip->SetNthLed(SubsetStripMap[i], Red_, Green_, Blue_);
    }
}
void PixelStripSubsetClass::SetColour(uint8_t Red_, uint8_t Green_, uint8_t Blue_, uint8_t Brightness_){
    for(int i=0; i < Length; i++){
        this->PixelStrip->SetNthLed(SubsetStripMap[i], Red_, Green_, Blue_, Brightness_);
    }
}
void PixelStripSubsetClass::SetBrightness(uint8_t Brightness_){
    ;
}
void PixelStripSubsetClass::SetNthLedColour(uint8_t Red_, uint8_t Green_, uint8_t Blue_){
    ;
}
void PixelStripSubsetClass::SetNthLedColour(uint8_t Red_, uint8_t Green_, uint8_t Blue_, uint8_t Brightness_){

}
void PixelStripSubsetClass::SetNthLedBrightness(uint8_t Brightness_){
    ;
}
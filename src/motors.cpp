/**
 * @file motors.c
 * @author Maciej Kurcius
 * @brief 
 * @version 0.1
 * @date 2021-12-16
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "motors.h"

MotorClass::MotorClass(uint32_t Pwm_pin_, TIM_TypeDef *Pwm_timer_, uint8_t PWM_tim_channel_, uint32_t Ilim_pin_, uint32_t A_channel_mot_,
             uint32_t B_channel_mot_, TIM_TypeDef *Enc_timer_, uint32_t A_channel_enc_, uint32_t B_channel_enc_, int8_t DefaultDir_){
    this->DefaultDir = DefaultDir_;
    this->PWM_pin = Pwm_pin_;
    this->PWM_tim_channel = PWM_tim_channel_;
    this->Ilim_pin = Ilim_pin_;
    this->A_channel_mot = A_channel_mot_;
    this->B_channel_mot = B_channel_mot_;
    this->A_channel_enc = A_channel_enc_;
    this->B_channel_enc = B_channel_enc_;
    this->Enc_tim = new HardwareTimer(Enc_timer_);
    this->Enc_tim->setMode(1, TIMER_INPUT_ENCODER_MODE12, A_channel_enc, B_channel_enc);
    this->Enc_tim->setOverflow(ENC_MAX_CNT);
    this->Enc_tim->refresh();
    this->Enc_tim->setCount(ENC_CNT_OFFSET);
    this->Enc_tim->resume();
    this->Enc_tim->getUnderOverFlow(ENC_MAX_CNT); // clear flag
    this->Pwm_tim = new HardwareTimer(Pwm_timer_);
    this->Pwm_tim->setMode(PWM_tim_channel_, TIMER_OUTPUT_COMPARE_PWM1, Pwm_pin_, 0);
    this->Pwm_tim->setOverflow(15000, HERTZ_FORMAT);
    this->Pwm_tim->setCaptureCompare(PWM_tim_channel, 0, PERCENT_COMPARE_FORMAT);
    this->Pwm_tim->resume();
    pinMode(this->A_channel_mot, OUTPUT_OPEN_DRAIN);
    pinMode(this->B_channel_mot, OUTPUT_OPEN_DRAIN);
    pinMode(this->Ilim_pin, OUTPUT_OPEN_DRAIN);
    // init values
    digitalWrite(this->A_channel_mot, HIGH);
    digitalWrite(this->B_channel_mot, HIGH);
    digitalWrite(this->Ilim_pin, HIGH);
}

MotorClass::~MotorClass(){
    ;
}

int64_t MotorClass::EncValUpdate(void){
    int8_t flag = Enc_tim->getUnderOverFlow(ENC_MAX_CNT);
    if(flag == 1)
        Enc_value += ENC_MAX_CNT;
    if(flag == -1)
        Enc_value -= ENC_MAX_CNT;
    return (Enc_value + Enc_tim->getCount()-ENC_CNT_OFFSET);
}

void MotorClass::SetPWM(uint16_t setpoint){
    int power = constrain(setpoint, 0, 100);
    this->Pwm_tim->setCaptureCompare(this->PWM_tim_channel, power, PERCENT_COMPARE_FORMAT);
}

void MotorClass::SetMove(int16_t vel){
    vel = vel;
    this->SetPWM(abs(vel));
    if(vel < 0){          //backward move
        digitalWrite(this->A_channel_mot, LOW);
        digitalWrite(this->B_channel_mot, HIGH);
    }
    else if(vel > 0){      //forward move
        digitalWrite(this->A_channel_mot, HIGH);
        digitalWrite(this->B_channel_mot, LOW);
    }
}

void MotorClass::EmgStop(void){
    digitalWrite(this->A_channel_mot, HIGH);
    digitalWrite(this->B_channel_mot, HIGH);
}

void MotorClass::SoftStop(void){
    digitalWrite(this->A_channel_mot, LOW);
    digitalWrite(this->B_channel_mot, LOW);
}

int16_t MotorClass::VelocityUpdate(uint16_t TimeChange){
    ActualEncVal = this->EncValUpdate();
    this->Velocity = int16_t((ActualEncVal-PrevEncVal)*1000/(IMP_PER_RAD)/TimeChange);
    // Serial.printf("Actual Value: %d, Previous Value: %d, Velocity: %d, LastTime: %d, NowTime: %d, Time Change: %d \r\n", 
    //             int32_t(ActEncVal), int32_t(PrevEncVal), int32_t(this->Velocity), LastTime, NowTime, TimeChange);
    PrevEncVal = ActualEncVal;
    return this->Velocity;
}

int16_t MotorClass::GetVelocity(void){
    return this->Velocity;
}

int8_t MotorClass::GetDefaultDir(void){
    return DefaultDir;
}

MotorPidClass::MotorPidClass(MotorClass* Motor_){
    Motor = Motor_;
    PID myPID(&Input, &Output, &PidSetpoint, Kp, Ki, Kd, P_ON_E, REVERSE);
    MotorPID = new PID(myPID);
    MotorPID->SetOutputLimits(OutputMin, OutputMax);
    MotorPID->SetSampleTime(1000/PID_FREQ);         
    MotorPID->SetMode(AUTOMATIC);
}

MotorPidClass::~MotorPidClass(){
    ;
}

void MotorPidClass::SetSetpoint(double Setpoint_){
    Setpoint = Setpoint_;
}

void MotorPidClass::Handler(void){
    ActualTime = xTaskGetTickCount();
    if(ActualSetpoint < Setpoint){
        if((ActualSetpoint + MAX_ACCELERATION) > Setpoint)
            ActualSetpoint = Setpoint;
        else
            ActualSetpoint += (MAX_ACCELERATION)/(PID_FREQ);
    }
    if(ActualSetpoint > Setpoint){
        if((ActualSetpoint - MAX_DECELERATION < Setpoint))
            ActualSetpoint = Setpoint;
        else
            ActualSetpoint -= (MAX_DECELERATION)/(PID_FREQ);
    }
    PidSetpoint = (ActualSetpoint*PID_MAX_OUTPUT)/(MAX_SPEED)*Motor->GetDefaultDir();
    Input = double((Motor->VelocityUpdate(ActualTime-PrevTime)*PID_MAX_OUTPUT)/(MAX_SPEED));
    //Serial.printf("Input: %d, Velocity: %d \r\n", int16_t(Input), int16_t(temp));
    MotorPID->Compute();
    Motor->SetMove(int16_t(this->Output));
    PrevTime = ActualTime;
    //Serial.printf("Actual setpoint: %d, Setpoint: %d, Output: %d, Input: %d \r\n", int16_t(this->ActualSetpoint), int16_t(this->Setpoint), int16_t(this->Output), int16_t(this->Input));
}

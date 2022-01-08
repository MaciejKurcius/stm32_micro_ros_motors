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
    // this->Pwm_tim->refresh();
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
    int8_t flag = this->Enc_tim->getUnderOverFlow(ENC_MAX_CNT);
    if(flag == 1){
        this->Enc_value += ENC_MAX_CNT;
    }
    if(flag == -1){
        this->Enc_value -= ENC_MAX_CNT;
    }
    return (this->Enc_value + this->Enc_tim->getCount()-ENC_CNT_OFFSET);
}

void MotorClass::SetPWM(uint16_t setpoint){
    int power = constrain(setpoint, 0, 100);
    this->Pwm_tim->setCaptureCompare(this->PWM_tim_channel, power, PERCENT_COMPARE_FORMAT);
}

void MotorClass::SetMove(int16_t vel){
    vel = vel*DefaultDir;
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

int16_t MotorClass::VelocityUpdate(void){
    static int64_t PrevEncVal = this->EncValUpdate();
    static int64_t ActEncVal = this->EncValUpdate();
    PrevEncVal = ActEncVal;
    ActEncVal = this->EncValUpdate();
    return this->Velocity = (ActEncVal - PrevEncVal)/(IMP_PER_RAD)*PID_FREQ;
}

int16_t MotorClass::GetVelocity(void){
    return this->Velocity;
}

MotorPidClass::MotorPidClass(MotorClass Motor_){
    this->Motor = new MotorClass(Motor_);
    this->Kp = PID_DEFAULT_KP;
    this->Ki = PID_DEFAULT_KI;
    this->Kd = PID_DEFAULT_KD;
    this->Error = 0;
    this->ActualValue = 0;
    this->MaxOutput = PID_MAX_OUTPUT;
    this->MinOutput = PID_MIN_OUTPUT;
    this->Setpoint = 0;
}

MotorPidClass::~MotorPidClass(){
    ;
}

void MotorPidClass::Init(void){
    ;
}

void MotorPidClass::Handler(void){
    if(Setpoint-ActualSetpoint > 0){
        if(abs(Setpoint-ActualSetpoint)>MAX_ACCELERATION)
            ActualSetpoint += MAX_ACCELERATION;
        else
            ActualSetpoint = Setpoint;
    }
    if(Setpoint-ActualSetpoint < 0){
        if(abs(Setpoint-ActualSetpoint)>MAX_ACCELERATION)
            ActualSetpoint -= MAX_DECELERATION;
        else
            ActualSetpoint = Setpoint;
    }
    ActualValue = this->Motor->VelocityUpdate();
    Error = ActualSetpoint - ActualValue;
    if(Error > MaxOutput){
        Output = MaxOutput;
    }
    else if(Error < MinOutput){
        Output = MinOutput;
    }
    else{
        Output = Error*Kp;
    }
    this->Motor->SetMove(55);
}
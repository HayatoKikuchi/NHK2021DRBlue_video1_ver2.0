#include "Button_Encorder.h"
#include "define.h"

// コンストラクタ
Button::Button(int xpin_num){
    pin_num = xpin_num;
    pinMode(pin_num, INPUT);
    pre_button_state = get_button_state();
    counts = 0;
    initial_flag = true;
}

DipSW::DipSW(){
    pinMode(PIN_DIP1,INPUT);
    pinMode(PIN_DIP2,INPUT);
    pinMode(PIN_DIP3,INPUT);
    pinMode(PIN_DIP4,INPUT);
}

Encorder::Encorder(){
    pinMode(PIN_ENC_A,INPUT);
    pinMode(PIN_ENC_B,INPUT);
    pinMode(PIN_LED_ENC, OUTPUT);

    pre_encA = 0;
    pre_encB = 0;
    count = 0.0;
    enc_count = 0;
    flag_enc = false;
}

bool Button::get_button_state(){
    return digitalRead(pin_num);
}

bool Button::button_rise(){
    bool ret = 0;
    bool button_state = get_button_state();
    if(initial_flag){
        if(button_state == 0){
            pre_button_state = button_state;
            initial_flag = false;
        }
    }else{
        if(button_state && !pre_button_state){
            initial_flag = true;
            ret = 1;
        }
        pre_button_state = button_state;
    }
    return ret;
}

bool Button::button_fall(){
    bool ret = 0;
    bool button_state = get_button_state();
    if(initial_flag){
        if(button_state == 1){
            pre_button_state = button_state;
            initial_flag = false;
        }
    }else{
        if(!button_state && pre_button_state){
            initial_flag = true;
            ret = 1;
        }
        pre_button_state = button_state;
    }
    return ret;
}

bool Button::button_changed(){
    bool ret = 0;
    bool button_state = get_button_state();
    if(initial_flag){
        pre_button_state = button_state;
        initial_flag = false;
    }else{
        if(button_state != pre_button_state){
            initial_flag = true;
            ret = 1;
        }
        pre_button_state = button_state;
    }
    return ret;
}

int Button::get_count(){
    if(button_fall()) counts++;
    return counts;
}

void Button::set_count(int setNum){
    counts = setNum;
}

int DipSW::getDipState(){
    dipNum = 0;
    if(!digitalRead(PIN_DIP1)) dipNum |= 0x01;
    if(!digitalRead(PIN_DIP2)) dipNum |= 0x02;
    if(!digitalRead(PIN_DIP3)) dipNum |= 0x04;
    if(!digitalRead(PIN_DIP4)) dipNum |= 0x08;

    return dipNum;
}

int Encorder::getEncCount(){
    if(flag_enc){
        encA = digitalRead(PIN_ENC_A);
        encB = digitalRead(PIN_ENC_B);
        if((encA != pre_encB) && (encB == pre_encA)) count -= 0.25;
        else if((encA == pre_encB) && (encB != pre_encA)) count += 0.25;
        pre_encA = encA;
        pre_encB = encB;
        enc_count = (int)count;
    }
    if(!flag_enc){
        pre_encA = digitalRead(PIN_ENC_A);
        pre_encB = digitalRead(PIN_ENC_B);
        flag_enc = true;
    }
    return enc_count;
}

void Encorder::setEncCount(int setnum){
    enc_count = setnum;
    count = (double)setnum;
}
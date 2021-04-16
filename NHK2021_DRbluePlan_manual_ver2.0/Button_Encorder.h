#ifndef _BUTTON_ENCORDER_H_INCLUDED
#define _BUTTON_ENCORDER_H_INCLUDED

#include <Arduino.h>

class Button{
  public:
  Button(int xpin_num);

  bool button_rise();
  bool button_fall();
  bool button_changed();
  bool get_button_state();
  int  get_count();
  void set_count(int setNum);

  private:
  int counts;
  int pin_num;
  bool pre_button_state;
  bool initial_flag;
};

class DipSW{
  public:
  DipSW();

  int getDipState();

  private:
  int dipNum;
};

class Encorder{
  public:
  
  Encorder();

  int getEncCount();
  void setEncCount(int setnum);
  int enc_count;
  double count;

  private:

  bool flag_enc;
  int encA, pre_encA, encB, pre_encB;
};

#endif
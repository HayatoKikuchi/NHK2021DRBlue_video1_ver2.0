#include "DRBlue.h"

DRBlue::DRBlue(lpms_me1 *_lpms, phaseCounter *_enc1, phaseCounter *_enc2)
{
  lpms = _lpms;
  enc1 = _enc1;
  enc2 = _enc2;

  pre_encX = 0.0;
  pre_encY = 0.0;
  position.x = 0.0;
  position.y = 0.0;
  position.z = 0.0;
  anlge_ofset = 0.0;
  setAngleNum = 0.0;
}

PIDsetting::PIDsetting(PID *_pid, myLCDclass *_LCD, Encorder *_encorder)
{
  pid = _pid;
  LCD = _LCD;
  encorder = _encorder;
}

DRexpand::DRexpand(byte _sw_pinName, byte _mosfet_phase1, byte _mosfet_phase2)
{
  sw_pinName = _sw_pinName;
  mosfet_phase1 = _mosfet_phase1;
  mosfet_phase2 = _mosfet_phase2;
  DRexpand::init();
}

DRwall::DRwall(byte pinSW, byte pinSupport, int MDadress, RoboClaw *_roboclaw) : sw(pinSW)
{
  adress = MDadress;
  pinSpt = pinSupport;
  roboclaw = _roboclaw;
  phase_1 = false;
  pinMode(pinSpt,OUTPUT);
  digitalWrite(pinSpt,HIGH);
}

/****自己位置推定の関数****/
void DRBlue::updateRobotPosition()
{
  
  encX_rad = (double)enc1->getCount() * _2PI_RES4;
  encY_rad = (double)enc2->getCount() * _2PI_RES4;
  roboAngle = ((double)lpms->get_z_angle() + anlge_ofset) + setAngleNum;

  encX = RADIUS_X * encX_rad;
  encY = RADIUS_Y * encY_rad;
  x_axis_prime = encX - pre_encX;
  y_axis_prime = encY - pre_encY;

  position.x += x_axis_prime*cos(roboAngle) - y_axis_prime*sin(roboAngle);
  position.y += x_axis_prime*sin(roboAngle) + y_axis_prime*cos(roboAngle);
  position.z = roboAngle;

  pre_encX = encX;
  pre_encY = encY;
}

void DRBlue::updateRoboAngle()
{
  roboAngle = (double)lpms->get_z_angle();
}

void DRBlue::setPosition(double x, double y, double z) // x[m]，y[m]，z[度]
{
  position.x = x;
  position.y = y;
  anlge_ofset = -1*position.z;
  position.z = roboAngle = setAngleNum = z/360.0 * 2.0 * PI_;
}

void DRBlue::DRsetup()
{
  pinMode(PIN_SUPPORT_RIGHT,OUTPUT);
  pinMode(PIN_SUPPORT_LEFT,OUTPUT);
  pinMode(PIN_EXPAND_RIGHT,OUTPUT);
  pinMode(PIN_EXPAND_LEFT,OUTPUT);
  pinMode(PIN_EXPAND_RIGHT,OUTPUT);
  pinMode(PIN_SW_EXPAMD_LEFT,INPUT);
  pinMode(PIN_SW_EXPAND_RIGHT,INPUT);
}

void DRBlue::BasicSetup()
{
    
  pinMode(PIN_SW, INPUT); // オンボードのスイッチ

  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);
  pinMode(PIN_LED_4, OUTPUT);
  pinMode(PIN_LED_USER, OUTPUT);

  LEDblink(PIN_LED_GREEN, 2, 100);
  //LPMS-ME1の初期化
  if(lpms->init() != 1){
    LEDblink(PIN_LED_BLUE, 3, 100);  // 初期が終わった証拠にブリンク
    delay(100);
  }
  
  enc1->init();
  enc2->init();
}

// LEDをチカチカさせるための関数
void DRBlue::LEDblink(byte pin, int times, int interval){
  analogWrite(pin, 0);
  for(int i = 0; i < times; i++){
    delay(interval);
    analogWrite(pin, 255);
    delay(interval);
    analogWrite(pin, 0);
  }
}

void DRBlue::RGB_led(int period){
  // RGB LED を良い感じに光らせるための処理
  static int count = 0;
  count += period; // ここで光る周期を変えられる(はず)
  if(count < 255){
    analogWrite(PIN_LED_RED, count);
    analogWrite(PIN_LED_BLUE, 255 - count);
  }
  else if(count < 255 * 2){
    analogWrite(PIN_LED_GREEN, count - 255);
    analogWrite(PIN_LED_RED, 255 * 2 - count);
  }
  else if(count < 255 * 3){
    analogWrite(PIN_LED_BLUE, count - 255 * 2);
    analogWrite(PIN_LED_GREEN, 255 * 3 - count);
  }
  else{
    count = 0;
  }
}

void DRBlue::allOutputLow(){
  digitalWrite(PIN_LED_1, LOW);
  digitalWrite(PIN_LED_2, LOW);
  digitalWrite(PIN_LED_3, LOW);
  digitalWrite(PIN_LED_4, LOW);
  digitalWrite(PIN_LED_ENC, LOW);

  digitalWrite(PIN_SUPPORT_RIGHT,LOW);
  digitalWrite(PIN_SUPPORT_LEFT,LOW);
  digitalWrite(PIN_EXPAND_RIGHT,LOW);
  digitalWrite(PIN_EXPAND_LEFT,LOW);
  digitalWrite(PIN_SUPPORT_WHEEL_1,LOW);
  digitalWrite(PIN_SUPPORT_WHEEL_2,LOW);
  digitalWrite(PIN_SUPPORT_WHEEL_3,LOW);
  digitalWrite(PIN_SUPPORT_WHEEL_4,LOW);
}


void PIDsetting::setting(double encorder_count, bool flag_500ms,bool up, bool down)
{
  static int pid_setting_mode = 1;
  static double Kp = 0.0, Ki = 0.0, Kd = 0.0;
  static bool init_kp = true, init_ki = true, init_kd = true;
  static bool flag_lcd = true;

  if(up)   pid_setting_mode++;
  else if(down) pid_setting_mode--;
  if(pid_setting_mode == 0) pid_setting_mode = 3;
  else if(pid_setting_mode == 4) pid_setting_mode = 1;
  
  if(flag_lcd)
  { 
    LCD->clear_display();
    LCD->write_str("RadianPID Setting",LINE_1,1);
    flag_lcd = false;
  }

  switch (pid_setting_mode)
  {
  case 1:
    init_ki = true;
    init_kd = true;
    if(init_kp)
    { 
      LCD->write_str("Kp ",LINE_3,1); //3コマ使用
      encorder->setEncCount((int)(10.0 * pid->Kp));
      init_kp = false;
    }
    Kp = 0.1*(double)encorder_count;
    if(flag_500ms)
    {
      LCD->write_str("          ",LINE_3,4);
      LCD->write_double(pid->Kp,LINE_3,4);
    }
    break;
  
  case 2:
    init_kp = true;
    init_kd = true;
    if(init_ki)
    {
      LCD->write_str("Ki ",LINE_3,1); //3コマ使用
      encorder->setEncCount((int)(10.0 * pid->Ki));
      init_ki = false;
    }
    Ki = 0.1*(double)encorder_count;
    if(flag_500ms)
    {
      LCD->write_str("          ",LINE_3,4);
      LCD->write_double(pid->Ki,LINE_3,4);
    }
    break;
  
  case 3:
    init_kp = true;
    init_ki = true;
    if(init_kd)
    {
      LCD->write_str("Kd ",LINE_3,1); //3コマ使用
      encorder->setEncCount((int)(10.0 * pid->Kd));
      init_kd = false;
    }
    Kd = 0.1*(double)encorder_count;
    if(flag_500ms)
    {
      LCD->write_str("          ",LINE_3,4);
      LCD->write_double(pid->Kd,LINE_3,4);
    }
    break;

  default:
    break;
  }
  pid->setPara(Kp,Ki,Kd);
}


void DRexpand::expand_func(bool ConButton, int mode) //モードは取り敢えず2
{
  if(expand.flag_pahse1)
  {
    if(ConButton)
    {
      digitalWrite(mosfet_phase1,HIGH);
      expand.flag_pahse1 = false;
    }
  }

  if(expand.flag_pahse2)
  {
    switch (mode)
    {
    case 1:
      if(!digitalRead(sw_pinName)) digitalWrite(mosfet_phase2,HIGH);
      break;
    
    case 2:
      if(ConButton) digitalWrite(mosfet_phase2,HIGH);
      break;

    default:
      break;
    }
  }else if(!expand.flag_pahse1) expand.flag_pahse2 = true;
}

void DRexpand::init(void)
{
  expand.flag_pahse1 = true;
  expand.flag_pahse2 = false;
  digitalWrite(mosfet_phase1,LOW);
  digitalWrite(mosfet_phase2,LOW);
}

int convert_position(double degree,double resorution, double gearration){
  double radian;
  radian = degree / 360.0 * 2.0 * PI_;
  return (int)((radian * 4.0*resorution)/(2.0*PI_) * gearration);
}

int convert_pps(double omega_deg, double resolution,double gearration)
{
  double omega_rad;
  omega_rad = omega_deg / 360.0 * 2.0 * PI_;
  return (int)(omega_rad / (2.0*PI_) * 4.0*resolution * gearration);
}

int convert_ppss(double accel, double resolution, double gearration)
{
  return (int)(accel / (2.0*PI_) * 4.0*resolution); // gearration);
}

void DRwall::wall_time_count(double int_time)
{
  if(wall_start) wall_time += int_time;
  else wall_time = 0.0;
}

bool DRwall::send_wall_position(double refAngle,double refOmega)
{
  position = convert_position(refAngle,RES_WALL,GEARRATIO_WALL);
  omega = convert_pps(refOmega,RES_WALL,GEARRATIO_WALL);
  accel = convert_ppss(KAKUKASOKUDO_WALL,RES_WALL,GEARRATIO_WALL);
  accel = 100000.0;
  roboclaw->SpeedAccelDeccelPositionM2(adress,accel,omega,accel,position,true);
  return true;
}

bool DRwall::send_wall_cmd(double robot_x_vel)
{ 
  bool send_cmd = false;
  static bool flag_cmd = true;
  if(sw.get_button_state() == 1) phase_1 = true;
  if(phase_1)
  {
    wall_start = true;
    digitalWrite(pinSpt,LOW);
    seconds = DISTANCE_WALL / robot_x_vel; // 90度回転するために必要な時間
    if(true)//(wall_time > 0.5)
    {
      if(flag_cmd)
      {
        DRwall::send_wall_position(90.0,270/seconds); // 90.0[deg]は倒立状態,180.0[deg]は回転角
        flag_cmd = false;
        send_cmd = true;
      }
      double target_seconds;
      target_seconds = position / omega;
      if((target_seconds - wall_time) < 0.01)
      {
        digitalWrite(pinSpt,HIGH);
        wall_start = false;
        phase_1 = false;
        flag_cmd = true;
      }  
    }
  }
  return send_cmd;
  send_cmd = false;
}

void DRwall::init()
{
  //send_wall_cmd(0.0,180.0);
}

void DRwall::roboclawSpeedM1(double vel)
{
  roboclaw->SpeedM1(adress,(int)vel);
}
void DRwall::roboclaw_begin(int baudlate)
{
  roboclaw->begin(baudlate);
}
void DRwall::roboclawResetEncoders()
{
  roboclaw->ResetEncoders(adress);
}
void DRwall::roboclawSpeedAccelM1(double accel, double speed)
{
  roboclaw->SpeedAccelM1(adress, (int)accel, (int)speed);
}
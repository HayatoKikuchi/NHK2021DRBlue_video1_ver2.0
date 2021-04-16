//NHK学生ロボコン2021 DR青プランのマニュアル制御
//最終更新　2021/02/24

#include <Arduino.h>
#include <MsTimer2.h>

#include "define.h"
#include "ManualControl.h"
#include "Controller.h"
#include "phaseCounterPeach.h"
#include "lpms_me1Peach.h"
#include "RoboClaw.h"
#include "PIDclass.h"
#include "DRBlue.h"
#include "Button_Encorder.h"
#include "LCDclass.h"
#include "Filter.h"

lpms_me1 lpms(&SERIAL_LPMSME1);
phaseCounter encX(1);
phaseCounter encY(2);
DRBlue DR(&lpms, &encX, &encY); //DRのセットアップなどを行う

PID posiZ_pid(2.5, 0.0, 1.0,INT_TIME);
ManualControl ManualCon(&posiZ_pid); //メカナムの速度制御

Controller Con(&SERIAL_XBEE); //dualshock4
myLCDclass lcd(&SERIAL_LCD);

RoboClaw roboclawR(&SERIAL_ROBOCLAW_R,1000);
RoboClaw roboclawL(&SERIAL_ROBOCLAW_L,1000);
DRwall wall_1(PIN_SW_WALL_1,PIN_SUPPORT_WHEEL_1,ADR_MD_WHEE_1,&roboclawL); // 壁越えの制御を行う
DRwall wall_2(PIN_SW_WALL_2,PIN_SUPPORT_WHEEL_2,ADR_MD_WHEE_2,&roboclawL); //　↓
DRwall wall_3(PIN_SW_WALL_3,PIN_SUPPORT_WHEEL_3,ADR_MD_WHEE_3,&roboclawR); //　↓
DRwall wall_4(PIN_SW_WALL_4,PIN_SUPPORT_WHEEL_4,ADR_MD_WHEE_4,&roboclawR); //　↓

DRexpand expand_right(PIN_SW_EXPAND_RIGHT,PIN_SUPPORT_RIGHT); // 展開機構
DRexpand expand_left(PIN_SW_EXPAMD_LEFT,PIN_SUPPORT_LEFT);    // ↓

Encorder enc; // 基板上のエンコーダ
int encorder_count; //エンコーダのカウント値を格納
DipSW dipsw; // 基板上のディップスイッチ
int dipsw_state; //ディップスイッチの状態を格納

/****基板上のスイッチ****/
Button button_red(PIN_SW_RED);
bool button_RED = false;
Button button_black(PIN_SW_BLACK);
bool button_BLACK = false;
Button button_up(PIN_SW_UP);
bool button_UP = false;
Button button_down(PIN_SW_DOWN);
bool button_DOWN = false;
Button button_right(PIN_SW_RIGHT);
bool button_RIGHT = false;
Button button_left(PIN_SW_LEFT);
bool button_LEFT = false;

/****変数****/
bool flag_10ms  = false;
bool flag_100ms = false;

int expand_mode; // 展開方法に関する変数
int robot_velocity_mode; //足回りの走行仕様について
int wall_mode; // 壁越え機構に関する変数
int turning_mode; //ロボットの旋回モード

double globalVelZ = 0.0;

void dipSetup()
{  
  if(dipsw_state & DIP1)
  {
    expand_mode = 2; // 2段階目をコントローラから指示する
    digitalWrite(PIN_LED_1,HIGH);
  }  
  else
  {
    digitalWrite(PIN_LED_1,LOW);
    expand_mode = 1; // 2段階目をリミットスイッチで判断
  } 

  if(dipsw_state & DIP2)
  {  
    robot_velocity_mode = 1; //X方向にのみ進む
    digitalWrite(PIN_LED_2,HIGH);
  }
  else
  {
    robot_velocity_mode = 2; //全方向移動
    digitalWrite(PIN_LED_2,LOW);
  }

  if(dipsw_state & DIP3)
  {
    wall_mode = 1; //テストモード
    digitalWrite(PIN_LED_3,HIGH);
  }
  else
  {
    wall_mode = 2; //通常モード
    digitalWrite(PIN_LED_3,LOW);
  }

  if(dipsw_state & DIP4)
  {
    turning_mode = 1; // 角度PID無し
    digitalWrite(PIN_LED_4,HIGH);
    digitalWrite(PIN_LED_ENC,LOW);
  }
  else
  {
    turning_mode = 2; // 角度PID有
    digitalWrite(PIN_LED_4,LOW);
    digitalWrite(PIN_LED_ENC,HIGH);
  }
}

void radianPID_setup(bool flag)
{
  static int pid_setting_mode = 1;
  static double Kp = 0.0, Ki = 0.0, Kd = 0.0;
  static bool init_kp = true, init_ki = true, init_kd = true;
  static bool flag_lcd = true;
  double serial_num;

  if(button_UP)   pid_setting_mode++;
  else if(button_DOWN) pid_setting_mode--;
  if(pid_setting_mode == 0) pid_setting_mode = 3;
  else if(pid_setting_mode == 4) pid_setting_mode = 1;
  
  if(flag_lcd && flag)
  { 
    lcd.clear_display();
    lcd.write_str("RadianPID Setting",LINE_1,1);
    flag_lcd = false;
  }

  switch (pid_setting_mode)
  {
  case 1:
    init_ki = true;
    init_kd = true;
    if(init_kp)
    { 
      if(flag) lcd.write_str("Kp ",LINE_3,1); //3コマ使用
      enc.setEncCount((int)(10.0 * posiZ_pid.Kp));
      init_kp = false;
    }
    Kp = 0.1*(double)encorder_count;
    if(flag_100ms && flag)
    {
      lcd.write_str("          ",LINE_3,4);
      lcd.write_double(posiZ_pid.Kp,LINE_3,4);
    }
    break;
  
  case 2:
    init_kp = true;
    init_kd = true;
    if(init_ki)
    {
      if(flag) lcd.write_str("Ki ",LINE_3,1); //3コマ使用
      enc.setEncCount((int)(10.0 * posiZ_pid.Ki));
      init_ki = false;
    }
    Ki = 0.1*(double)encorder_count;
    if(flag_100ms && flag)
    {
      lcd.write_str("          ",LINE_3,4);
      lcd.write_double(posiZ_pid.Ki,LINE_3,4);
    }
    break;
  
  case 3:
    init_kp = true;
    init_ki = true;
    if(init_kd)
    {
      if(flag) lcd.write_str("Kd ",LINE_3,1); //3コマ使用
      enc.setEncCount((int)(10.0 * posiZ_pid.Kd));
      init_kd = false;
    }
    Kd = 0.1*(double)encorder_count;
    if(flag_100ms && flag)
    {
      lcd.write_str("          ",LINE_3,4);
      lcd.write_double(posiZ_pid.Kd,LINE_3,4);
    }
    break;

  default:
    break;
  }
  /*
  if(flag_100ms && flag)
  {
    lcd.write_str("        ",LINE_4,7);
    lcd.write_double(serial_num,LINE_4,7);
  }
  */
  posiZ_pid.setPara(Kp,Ki,Kd);
}

void SendAllWheelPosition(double deg, double refOmega)
{
  wall_1.send_wall_position(deg, refOmega);
  wall_2.send_wall_position(deg, refOmega);
  wall_3.send_wall_position(deg, refOmega);
  wall_4.send_wall_position(deg, refOmega);
}

// 最大最小範囲に収まるようにする関数
double min_max(double value, double minmax)
{
  if(value > minmax) value = minmax;
  else if(value < -minmax) value = -minmax;
  return value;
}
/****割込みの関数****/
void timer_warikomi()
{
  DR.RGB_led(2);
  //DR.updateRobotPosition();
  DR.updateRoboAngle(); // updateRobotPosition関数を使用する場合は不要
  ManualCon.updatePosiPID(globalVelZ,MAXOMEGA,DR.roboAngle,JOYCONPID);

  wall_1.wall_time_count(INT_TIME); // 壁越えの時間に関する処理
  wall_2.wall_time_count(INT_TIME); // ↓
  wall_3.wall_time_count(INT_TIME); // ↓
  wall_4.wall_time_count(INT_TIME); // ↓

  encorder_count = enc.getEncCount(); //エンコーダのカウント値を更新
  dipsw_state = dipsw.getDipState(); // ディップスイッチの状態を更新
  button_RED = button_red.button_fall();
  button_BLACK = button_black.button_fall();
  button_UP = button_up.button_fall();
  button_DOWN = button_down.button_fall();
  button_RIGHT = button_right.button_fall();
  button_LEFT = button_left.button_fall();

  static int count_100ms = 0;
  if(10 <= count_100ms){
    flag_100ms = true;
    count_100ms = 0;
  }
  count_100ms++;

  flag_10ms = true;
}

void setup()
{
  delay(1000);
  Serial.begin(115200);
  //SERIAL_LEONARDO.begin(9600);
  SERIAL_LCD.begin(115200);

  DR.DRsetup();      //　汎用基板などの様々なセットアップを行う
  DR.BasicSetup();   //　汎用基板などの様々なセットアップを行う
  DR.allOutputLow(); //  出力をLOWにする

  roboclawR.begin(230400);
  roboclawL.begin(230400);

  Con.begin(115200);
  //Con.begin_api(115200); // apiでの通信

  lcd.clear_display();
  lcd.color_red();
  lcd.write_str("please initialize",LINE_2,1);
  
  //ボードのスイッチが押されるまで待機
  bool ready_to_start = false;
  bool wall_init = false;
  while(!ready_to_start)
  {
    Con.update(PIN_LED_USER);
    //roboclawの原点出し
    if(Con.readButton(BUTTON_PS,PUSHED) || !digitalRead(PIN_SW))
    {  
      roboclawL.ResetEncoders(ADR_MD_WHEE_1);
      roboclawL.ResetEncoders(ADR_MD_WHEE_2);
      roboclawR.ResetEncoders(ADR_MD_WHEE_3);
      roboclawR.ResetEncoders(ADR_MD_WHEE_4);
      posiZ_pid.PIDinit(0.0,0.0);
      DR.LEDblink(PIN_LED_GREEN, 2, 100);
      lcd.clear_display();
      lcd.write_str("push triangle",LINE_3,1);
      wall_init = true;
    }

    if((Con.readButton(BUTTON_PS,PUSHED) || !digitalRead(PIN_SW)) && wall_init)
    {
      static bool once_loop = false; //BUTTON_PSのPUSHEDを一度見送る
      if(once_loop)
      {
        DR.LEDblink(PIN_LED_BLUE, 2, 100);
        lcd.clear_display();
        lcd.color_blue();
        lcd.clear_display();

        lcd.write_str("  HELLOW WORLD   ",LINE_2,1);
        delay(1000);
        ready_to_start = true;
      }
      once_loop = true;
    }
  }

  MsTimer2::set(10,timer_warikomi); // 10ms period
  MsTimer2::start();
  DR.setPosition(0.0,0.0,0.0);
}

void loop()
{
  Con.update(PIN_LED_USER); //コントローラの状態を更新
  dipSetup(); // ディップスイッチでの設定
/*
  uint8_t led_num = 0;
  while(SERIAL_LEONARDO.available()){
    char recv =  SERIAL_LEONARDO.read();
    if(recv != '\n'){
      led_num = (uint8_t)recv;
      digitalWrite(PIN_LED_1, led_num & 0x01);
      digitalWrite(PIN_LED_2, led_num & 0x02);
      digitalWrite(PIN_LED_3, led_num & 0x04);
      digitalWrite(PIN_LED_4, led_num & 0x08);
      Serial.println(led_num);
    }
  }
*/
  //展開前の状態に戻す
  if(Con.readButton(BUTTON_PAD,PUSHED))
  {
    expand_right.init();
    expand_left.init();
    wall_1.init();
    wall_2.init();
    wall_3.init();
    wall_4.init();
  }

  //**10msの処理**//
  if( flag_10ms )
  {
    //旋回に関する設定//
    if(turning_mode == 2 ) radianPID_setup(false); // pidのゲインを設定
    else radianPID_setup(true);

    //展開に関する処理//
    //expand_right.expand_func(Con.readButton(BUTTON_R1,PUSHED),expand_mode); //展開右
    //expand_left.expand_func(Con.readButton(BUTTON_L1,PUSHED),expand_mode);  //展開左

    static double Cx = 1.0;
    static double Cy = 1.0;
    static double Cz = 1.0; //速度の倍数
    
    if(Con.readButton(BUTTON_R2,PUSHED))
    { 
      if(Cx < 3.0) //上限は3倍(3.0m/s)
      {
        Cx += 0.5; 
        Cy += 0.5;
      }
      lcd.clear_display();
      lcd.write_double(Cx,LINE_3,3);
    }
    if(Con.readButton(BUTTON_L2,PUSHED))
    {
      if(0.5 < Cx) //下限は0.5倍(0.5m/s)
      {
        Cx -= 0.5;
        Cy -= 0.5;
      }
      lcd.clear_display();
      lcd.write_double(Cy,LINE_3,3);
    }
    static double robotRefDeg = 0.0;
    if(Con.readButton(BUTTON_SIKAKU,PUSHED))
    {
      robotRefDeg += 90.0;
      ManualCon.setRefAngle(robotRefDeg);
    }
    if(Con.readButton(BUTTON_MARU,PUSHED))
    {
      robotRefDeg -= 90.0;
      ManualCon.setRefAngle(robotRefDeg);
    }
    
    coords gloabalVel = ManualCon.getGlobalVel(Con.LJoyX,Con.LJoyY,Con.RJoyY);
    globalVelZ = gloabalVel.z;
    coords localVel = ManualCon.getLocalVel(Cx*gloabalVel.x, Cy*gloabalVel.y, Cz*gloabalVel.z, DR.roboAngle, JOYCONPID);
    // coords refV = ManualCon.getVel_max(localVel.x, localVel.y, localVel.z);
    // coords VelCmd = ManualCon.getCmd(refV.x,refV.y,refV.z);
    coords_4 VelCmd = ManualCon.getCmd(localVel.x,localVel.y,localVel.z);

    wall_1.roboclawSpeedM1(VelCmd.i);   // 足回りの速度指令
    wall_2.roboclawSpeedM1(VelCmd.ii);  // ↓
    wall_3.roboclawSpeedM1(VelCmd.iii); // ↓
    wall_4.roboclawSpeedM1(VelCmd.iv);  // ↓

    switch (wall_mode)
    {
    case 1:
      if(Con.readButton(BUTTON_RIGHT,PUSHED)) SendAllWheelPosition(-90.0,180.0);  //倒立状態
      if(Con.readButton(BUTTON_UP,PUSHED))    SendAllWheelPosition(0.0,180.0);    //初期状態（8輪接地）
      if(Con.readButton(BUTTON_LEFT,PUSHED))  SendAllWheelPosition(90.0,180.0);   //壁越え後の倒立状態
      if(Con.readButton(BUTTON_DOWN,PUSHED))  SendAllWheelPosition(180.0,180.0);  //壁越え後の8輪接地

      break;

      default:
       break;
    }
    wall_1.send_wall_cmd(Cx);
    wall_2.send_wall_cmd(Cx);
    wall_3.send_wall_cmd(Cx);
    wall_4.send_wall_cmd(Cx);
    if(Con.readButton(BUTTON_SANKAKU,PUSHED)) SendAllWheelPosition(180.0,180.0); //壁越え後の8輪接地

    flag_10ms = false;
  }
  if(flag_100ms)
  {
    flag_100ms = false;
  }
}
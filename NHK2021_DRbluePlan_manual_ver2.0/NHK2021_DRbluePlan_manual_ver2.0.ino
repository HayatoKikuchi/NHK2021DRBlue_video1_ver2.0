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

PID posiX_pid(0.0,0.0,0.0,INT_TIME);
PID posiY_pid(0.0,0.0,0.0,INT_TIME);
PID posiZ_pid(2.5, 0.0, 1.0,INT_TIME);
ManualControl ManualCon(&posiZ_pid); //メカナムの速度制御

Controller Con(&SERIAL_XBEE); //dualshock4
myLCDclass lcd(&SERIAL_LCD);

RoboClaw roboclawL(&SERIAL_ROBOCLAW_L,1000);
RoboClaw roboclawR(&SERIAL_ROBOCLAW_R,1000);
DRwall wall_1(PIN_SW_WALL_1,PIN_SUPPORT_WHEEL_1,ADR_MD_WHEE_1,&roboclawL); // 壁越えの制御を行う
DRwall wall_2(PIN_SW_WALL_2,PIN_SUPPORT_WHEEL_2,ADR_MD_WHEE_2,&roboclawL); //　↓
DRwall wall_3(PIN_SW_WALL_3,PIN_SUPPORT_WHEEL_3,ADR_MD_WHEE_3,&roboclawR); //　↓
DRwall wall_4(PIN_SW_WALL_4,PIN_SUPPORT_WHEEL_4,ADR_MD_WHEE_4,&roboclawR); //　↓

//DRexpand expand_right(PIN_SW_EXPAND_RIGHT,PIN_SUPPORT_RIGHT); // 展開機構
DRexpand expand_left(PIN_SW_EXPAMD_LEFT,PIN_SUPPORT_LEFT,PIN_EXPAND_LEFT);    // ↓

Encorder enc; // 基板上のエンコーダ
int encorder_count; //エンコーダのカウント値を格納
DipSW dipsw; // 基板上のディップスイッチ
int dipsw_state; //ディップスイッチの状態を格納

PIDsetting posiXpidSetting(&posiX_pid, &lcd, &enc);
PIDsetting posiYpidSetting(&posiY_pid, &lcd, &enc);
PIDsetting posiZpidSetting(&posiZ_pid, &lcd, &enc);

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
double globalVelZ;

void radianPID_setup(bool flag)
{
  static int pid_setting_mode = 1;
  static double Kp = 0.0, Ki = 0.0, Kd = 0.0;
  static bool init_kp = true, init_ki = true, init_kd = true;
  static bool flag_lcd = true;

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
  DR.updateRobotPosition();
  //DR.updateRoboAngle(); // updateRobotPosition関数を使用する場合は不要
  ManualCon.updatePosiPID(globalVelZ,MAXOMEGA,DR.roboAngle,JOYCONPID);

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
  SERIAL_LPC1768.begin(115200);
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
  lcd.write_str("----Setting Time----",LINE_2,0);
  lcd.write_str("PUSH BUTTON_PS",LINE_3,1);
  
  //ボードのスイッチが押されるまで待機
  bool ready_to_start = false;
  bool wall_init = false;
  while(!ready_to_start)
  {
    Con.update(PIN_LED_USER);
    //roboclawの原点出し
    if(Con.readButton(BUTTON_PS,PUSHED) || !digitalRead(PIN_SW))
    {  
      posiZ_pid.PIDinit(0.0,0.0);
      DR.LEDblink(PIN_LED_GREEN, 2, 100);
      lcd.clear_display();
      lcd.write_str("Are you ready?",LINE_2,1);
      lcd.write_str("PUSH BUTTON_PS",LINE_3,1);
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

        delay(1000);
        lcd.write_str("  HELLOW WORLD   ",LINE_2,1);
        ready_to_start = true;
      }
      once_loop = true;
    }
  }

  MsTimer2::set(10,timer_warikomi); // 10ms period
  MsTimer2::start();
  DR.setPosition(0.5,-0.5,0.0);
}

void loop()
{
  //**LPC1768にコントローラの情報を送る処理**//
  bool lpc_com; //LPC1768にコントローラのデータを送信するかしないか
  lpc_com = Con.update(PIN_LED_USER); //コントローラの状態を更新
  Con.setAvailable(dipsw_state & DIP1_CON);
  if(lpc_com && (!Con.ConAvailable))//GR-PEACHがコントローラのデータを使用できないときに送信
  {
    for(int i = 0; i < DATA_NUM;i++) SERIAL_LPC1768.print(Con.raww_recv_msgs[i]);
    SERIAL_LPC1768.println();
  }


  //**展開機構の処理**//
  expand_left.expand_func(Con.readButton(BUTTON_L1,PUSHED),2);
  if(Con.readButton(BUTTON_PS,PUSHED)) expand_left.init();


  //**最高速度の変更*///
  static double Cx = 1.0; //速度の倍数
  static double Cy = 1.0; //速度の倍数
  static double Cz = 1.0; //速度の倍数 

  if(button_UP)//ボード上のスイッチ
  { 
    if(Cx < 3.0) //上限は3倍(3.0m/s)
    {
      Cx += 0.5; 
      Cy += 0.5;
    }
  }
  if(button_DOWN)//ボード上のスイッチ
  {
    if(0.5 < Cx) //下限は0.5倍(0.5m/s)
    {
      Cx -= 0.5;
      Cy -= 0.5;
    }
  }

  
  //**旋回角度の指定**//
  static double robotRefDeg = 0.0;
  if(Con.readButton(BUTTON_LEFT,PUSHED))
  {
    robotRefDeg += 90.0;
    ManualCon.setRefAngle(robotRefDeg);
  }
  if(Con.readButton(BUTTON_RIGHT,PUSHED))
  {
    robotRefDeg -= 90.0;
    ManualCon.setRefAngle(robotRefDeg);
  }


  //**10msの処理**//
  if( flag_10ms )
  {
    //旋回に関する設定//
    if(dipsw_state & DIP4_PIDSETTING) radianPID_setup(true); // pidのゲインを設定
    
    if(dipsw_state & DIP4_PIDSETTING)
    {
      static int pid_setting_num = 1;
      if(button_RIGHT)
      {
        pid_setting_num++;
        if(4 <= pid_setting_num) pid_setting_num = 1;
      }
      else if(button_LEFT)
      {
        pid_setting_num--;
        if(pid_setting_num <= 0) pid_setting_num = 3;
      }

      switch (pid_setting_num)
      {
      case 1: posiXpidSetting.setting(encorder_count,flag_100ms,button_UP,button_DOWN); break;
      case 2: posiYpidSetting.setting(encorder_count,flag_100ms,button_UP,button_DOWN); break;
      case 3: posiZpidSetting.setting(encorder_count,flag_100ms,button_UP,button_DOWN); break;
      
      default:
        break;
      }
    }
    
    
    coords gloabalVel = ManualCon.getGlobalVel(Con.readJoy(LX),Con.readJoy(LY),Con.readJoy(RY));
    globalVelZ = gloabalVel.z;
    coords localVel = ManualCon.getLocalVel(Cx*gloabalVel.x, Cy*gloabalVel.y, Cz*gloabalVel.z, DR.roboAngle, JOYCONPID);
    // coords refV = ManualCon.getVel_max(localVel.x, localVel.y, localVel.z);
    // coords VelCmd = ManualCon.getCmd(refV.x,refV.y,refV.z);
    coords_4 VelCmd = ManualCon.getCmd(localVel.x,localVel.y,localVel.z);

    wall_1.roboclawSpeedM1(VelCmd.i);   // 足回りの速度指令
    wall_2.roboclawSpeedM1(VelCmd.ii);  // ↓
    wall_3.roboclawSpeedM1(VelCmd.iii); // ↓
    wall_4.roboclawSpeedM1(VelCmd.iv);  // ↓

    flag_10ms = false;
  }


  //**100msの処理（主にLCDの更新に使用）**//
  if(flag_100ms)
  { 
    lcd.clear_display();
    lcd.write_str("X:",LINE_1,1);
    lcd.write_str("Y:",LINE_2,1);
    lcd.write_str("Z:",LINE_3,1);
    lcd.write_str("V:",LINE_4,1);
    lcd.write_double(DR.position.x,LINE_1,4);
    lcd.write_double(DR.position.y,LINE_2,4);
    lcd.write_double(DR.position.z/(2.0*PI_)*360.0,LINE_3,4);
    lcd.write_double(Cx,LINE_4,4);

    Serial.println(Con.getButtonState());

    flag_100ms = false;
  }


  //**ボード上のスイッチ（DIPスイッチに対応）**//
  digitalWrite(PIN_LED_1,dipsw_state & DIP1_CON);
  digitalWrite(PIN_LED_2,dipsw_state & DIP2);
  digitalWrite(PIN_LED_3,dipsw_state & DIP3);
  digitalWrite(PIN_LED_4,dipsw_state & DIP4_PIDSETTING);
}
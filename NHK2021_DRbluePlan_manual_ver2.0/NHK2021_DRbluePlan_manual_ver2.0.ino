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
//DRexpand expand_left(PIN_SW_EXPAMD_LEFT,PIN_SUPPORT_LEFT,PIN_EXPAND_LEFT);    // ↓

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
bool flag_500ms = false;
bool inner_area = false;
coords GlobalVelPID = {0.0, 0.0, 0.0};
coords refPIDsetting = {0.0,0.0,0.0};
double vel_max;

//**指定したエリア内にいるかいないか**//
bool area(double point,double min,double max)
{
  if(min < point && point < max) return true;
  return false;
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
  ManualCon.updatePosiPID(GlobalVelPID.z,MAXOMEGA,DR.roboAngle,JOYCONPID);
  if(dipsw_state & DIP4_PIDSETTING)
  {
    GlobalVelPID.x = posiX_pid.getCmd(refPIDsetting.x,DR.position.x,vel_max);
    GlobalVelPID.y = posiY_pid.getCmd(refPIDsetting.y,DR.position.y,vel_max);
  }
  else if(dipsw_state & DIP2_POT_PID)
  {
    static bool posi_phase1 = true;
    static bool posi_phase2 = false;
    if((4.925 < DR.position.x) && (-4.0 < DR.position.y) && posi_phase1)
    {
      GlobalVelPID.x = posiX_pid.getCmd(4.149,DR.position.x,vel_max);
      GlobalVelPID.y = posiY_pid.getCmd(-3.402,DR.position.y,vel_max);
      inner_area = true;
      posi_phase1 = false;
      posi_phase2 = true;
    }
    if(area(DR.position.x,4.149-0.05,4.149+0.05) && area(DR.position.y,-3.402-0.05,-3.402+0.05) && posi_phase2)
    {
      GlobalVelPID.x = posiX_pid.getCmd(4.149,DR.position.x,vel_max);
      GlobalVelPID.y = posiY_pid.getCmd(-3.402,DR.position.y,vel_max);
      posi_phase2 = false;
    }
  }


  encorder_count = enc.getEncCount(); //エンコーダのカウント値を更新
  dipsw_state = dipsw.getDipState(); // ディップスイッチの状態を更新
  button_RED = button_red.button_fall();
  button_BLACK = button_black.button_fall();
  button_UP = button_up.button_fall();
  button_DOWN = button_down.button_fall();
  button_RIGHT = button_right.button_fall();
  button_LEFT = button_left.button_fall();

  static int count_500ms = 0;
  if(50 <= count_500ms){
    flag_500ms = true;
    count_500ms = 0;
  }
  count_500ms++;

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
        lcd.color_green();
        lcd.clear_display();

        delay(1000);
        lcd.write_str("  HELLO WORLD   ",LINE_2,1);
        ready_to_start = true;
      }
      once_loop = true;
    }
  }

  MsTimer2::set(10,timer_warikomi); // 10ms period
  MsTimer2::start();
  ManualCon.setRefAngle(-90.0);
  DR.setPosition(0.5,-0.5,-90.0);
}

void loop()
{
  //**LPC1768にコントローラの情報を送る処理**//
  bool lpc_com; //LPC1768にコントローラのデータを送信するかしないか
  lpc_com = Con.update(PIN_LED_USER); //コントローラの状態を更新
  if(lpc_com && (!Con.ConAvailable))//GR-PEACHがコントローラのデータを使用できないときに送信
  {
    for(int i = 0; i < DATA_NUM;i++) SERIAL_LPC1768.print(Con.raww_recv_msgs[i]);
    SERIAL_LPC1768.println();
    digitalWrite(PIN_LED_1,LOW);
  }
  else if(Con.ConAvailable)
  {
    digitalWrite(PIN_LED_1,HIGH);
  }


  //**展開機構の処理**//
  //expand_left.expand_func(Con.readButton(BUTTON_L1,PUSHED),2);
  //if(Con.readButton(BUTTON_PS,PUSHED)) expand_left.init();


  //**10msの処理**//
  if( flag_10ms )
  {  
    //旋回に関する設定//
    //if(dipsw_state & DIP4_PIDSETTING) radianPID_setup(true); // pidのゲインを設定
    
    Con.setAvailable(dipsw_state & DIP1_CON);
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
      case 1: posiXpidSetting.setting(encorder_count,flag_500ms,button_UP,button_DOWN); break;
      case 2: posiYpidSetting.setting(encorder_count,flag_500ms,button_UP,button_DOWN); break;
      case 3: posiZpidSetting.setting(encorder_count,flag_500ms,button_UP,button_DOWN); break;
      
      default:
        break;
      }
    }

    // //**展開機構の処理**//
    if(Con.readButton(BUTTON_SANKAKU,PUSHE))  digitalWrite(PIN_SUPPORT_LEFT,HIGH);
    //if((4.5 < DR.position.x) && (-3.0 < DR.position.y) && (dipsw_state & DIP3_AUTO2)) digitalWrite(PIN_SUPPORT_LEFT,HIGH);
  
    //**旋回角度の指定**//
    static double robotRefDeg = -90.0;
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
    static bool change_roboAngle = true;
    if((4.0 < DR.position.x) && (-3.5 < DR.position.y) && change_roboAngle && (dipsw_state & DIP3_AUTO2))
    {
      ManualCon.setRefAngle(0.0);
      change_roboAngle = false;
    } 

    //**最高速度の変更*///
    static double Cx = 1.0; //速度の倍数
    static double Cy = 1.0; //速度の倍数
    static double Cz = 1.0; //速度の倍数
    //static int Cxy_stock = 1.0;
    if(~dipsw_state & DIP4_PIDSETTING)
    {
      if(button_UP)//ボード上のスイッチ
      { 
        if(Cx < 3.0) //上限は3倍(3.0m/s)
        {
          Cx += 0.5; 
          Cy += 0.5;
          //Cxy_stock = Cx;     
        }
      }
      if(button_DOWN)//ボード上のスイッチ
      {
        if(0.5 < Cx) //下限は0.5倍(0.5m/s)
        {
          Cx -= 0.5;
          Cy -= 0.5;
          //Cxy_stock = Cy;
        }
      }
    }
    // if(Con.readButton(BUTTON_L2,PUSHE)) Cx = Cy = 0.3;
    // else Cx = Cy = Cxy_stock;
    vel_max = Cx;
    coords localVel;
    coords gloabalVel = ManualCon.getGlobalVel(Con.readJoy(LX),Con.readJoy(LY),Con.readJoy(RY));
    GlobalVelPID.z = gloabalVel.z;
    if(~dipsw_state & DIP2_POT_PID && !inner_area)  localVel = ManualCon.getLocalVel(Cx*gloabalVel.x, Cy*gloabalVel.y, Cz*gloabalVel.z, DR.roboAngle, JOYCONPID);
    else if(dipsw_state & DIP2_POT_PID && inner_area) localVel = ManualCon.getLocalVel(GlobalVelPID.x, GlobalVelPID.y,GlobalVelPID.z, DR.roboAngle, POSITIONPID);

    //**PIDsetting**//
    static double stock_posiX, stock_posiY;
    static bool PIDSetting_phase1 = true;
    static bool PIDSetting_phase2 = false;
    if(dipsw_state & DIP4_PIDSETTING)
    {
      if(PIDSetting_phase1)
      {
        stock_posiX = DR.position.x;
        stock_posiY = DR.position.y;
        DR.setPosition(0.0,0.0,DR.roboAngle);
        PIDSetting_phase1 = false;
        PIDSetting_phase2 = true;
      }
      else
      {
        if(Con.readButton(BUTTON_R1,PUSHED)) refPIDsetting.x = refPIDsetting.y = 1.0;
        if(Con.readButton(BUTTON_L1,PUSHED)) refPIDsetting.x = refPIDsetting.y = 0.0;
      }
    }
    else
    {
      if(PIDSetting_phase2)
      {
        DR.setPosition(stock_posiX+DR.position.x,stock_posiY+DR.position.y,DR.roboAngle);
        PIDSetting_phase1 = true;
        PIDSetting_phase2 = false;
      }
    }
    coords_4 VelCmd = ManualCon.getCmd(localVel.x,localVel.y,localVel.z);

    wall_1.roboclawSpeedM1(VelCmd.i);   // 足回りの速度指令
    wall_2.roboclawSpeedM1(VelCmd.ii);  // ↓
    wall_3.roboclawSpeedM1(VelCmd.iii); // ↓
    wall_4.roboclawSpeedM1(VelCmd.iv);  // ↓

    flag_10ms = false;
  }


  //**500msの処理（主にLCDの更新に使用）**//
  if(flag_500ms)
  { 
    lcd.clear_display();
    
    lcd.write_str("X:",LINE_1,1);
    lcd.write_str("Y:",LINE_2,1);
    lcd.write_str("Z:",LINE_3,1);
    lcd.write_str("VelMax:",LINE_4,1);
    lcd.write_double(DR.position.x,LINE_1,4);
    lcd.write_double(DR.position.y,LINE_2,4);
    lcd.write_double(DR.position.z/(2.0*PI_)*360.0,LINE_3,4);
    lcd.write_double(vel_max,LINE_4,9);

    flag_500ms = false;
  }

  //**ボード上のスイッチ（DIPスイッチに対応）**//
  digitalWrite(PIN_LED_2,dipsw_state & DIP2_POT_PID);
  digitalWrite(PIN_LED_3,dipsw_state & DIP3_AUTO2);
  digitalWrite(PIN_LED_4,dipsw_state & DIP4_PIDSETTING);
}
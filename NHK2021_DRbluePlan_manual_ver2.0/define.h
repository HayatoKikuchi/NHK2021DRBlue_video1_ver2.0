#ifndef DEFINE_h
#define DEFINE_h

#include "Arduino.h"

struct coords{
    double x;
    double y;
    double z;
};

struct coords_4{
    double i;
    double ii;
    double iii;
    double iv;
};

// スイッチやLEDのピン設定
#define PIN_SW_UP    30 //32でした
#define PIN_SW_LEFT  31 //33でした
#define PIN_SW_RIGHT 33 //31でした
#define PIN_SW_DOWN  32 //30でした
#define PIN_ENC_A  26
#define PIN_ENC_B  27
#define PIN_DIP1 25
#define PIN_DIP2 24
#define PIN_DIP3 69
#define PIN_DIP4 70
#define DIP1_CON 0x01
#define DIP2_POT_PID 0x02
#define DIP3_AUTO2 0x04
#define DIP4_PIDSETTING 0x08

#define PIN_SW_BLACK  29
#define PIN_SW_RED    28

#define PIN_LED_1   20
#define PIN_LED_2   36
#define PIN_LED_3   37
#define PIN_LED_4   38
#define PIN_LED_ENC 40

#define PIN_SUPPORT_RIGHT 47
#define PIN_SUPPORT_LEFT 46
#define PIN_EXPAND_RIGHT 45
#define PIN_EXPAND_LEFT 44
#define PIN_SUPPORT_WHEEL_1 48
#define PIN_SUPPORT_WHEEL_2 49
#define PIN_SUPPORT_WHEEL_3 50
#define PIN_SUPPORT_WHEEL_4 51

#define PIN_SW_WALL_1 43
#define PIN_SW_WALL_2 53
#define PIN_SW_WALL_3 4
#define PIN_SW_WALL_4 5
#define PIN_SW_EXPAND_RIGHT 3
#define PIN_SW_EXPAMD_LEFT 2

//#define SERIAL_LPC1768  Serial0
#define SERIAL_LPMSME1  Serial1
#define SERIAL_ROBOCLAW_L Serial0 //wheel1 and wheel2
#define SERIAL_ROBOCLAW_R Serial4 //wheel3 and wheel4
#define SERIAL_LPC1768 Serial5
#define SERIAL_LCD      Serial6
#define SERIAL_XBEE     Serial7

// 制御周期
#define INT_TIME			( 0.01 )//( 0.001 )
#define PI_ ( 3.1415926 )

// 自己位置推定用エンコーダ関連
#define _2PI_RES4   ( 2 * PI_ / (4*100) ) // 分解能 100ppr
#define RADIUS_X    ( 0.0375/2.0 )    
#define RADIUS_Y    ( 0.0375/2.0 )

#define DRIVE_MECHANUM      ( 0 )
#define DRIVE_OMNI4WHEEL    ( 1 )
#define DRIVE_OMNI3WHEEL    ( 2 )
#define DRIVE_DUALWHEEL     ( 3 )
#define DRIVE_MECHANUM_8WHEELS     ( 4 )

#define DRIVE_MODE  ( DRIVE_MECHANUM )

#if DRIVE_MODE == DRIVE_DRIVE_MECHANUM
    // 4輪メカナム関連
    #define RES_MECHANUM			( 1024.0 ) // 1024pulses
    #define RES_WALL                ( 1024.0 ) // 1024pulses
    #define MECANUM_HANKEI		( 0.127 / 2.0 ) // 駆動輪の半径
    #define MECHANUM_A	( 0.573/2.0 ) // 外側のホイール a[m]
    #define MECHANUM_B	( 0.540/2.0 ) // 外側のホイール b[m]
    #define _4RES_2PI_WHEEL ( 4 * RES_MECHANUM / ( 2.0*PI_ ) ) // 駆動輪の角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数(1024ppr)
    #define _4RES_2PI_WALL ( 4 * RES_WALL / ( 2.0*PI_ ) ) // 壁越え機構の角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数(1024ppr)
    #define GEARRATIO_WHEEL ( 1.0 ) // 駆動輪のギヤ比
    #define GEARRATIO_WALL ( 3.0 )// 壁越え機構ギヤ比
    #define MAXVEL ( 1.0 )
    #define MAXOMEGA ( PI_ / 3.0 * 2.0 ) // 2/3π [rad/s]
    #define KAKUKASOKUDO_WALL ( 471.2389 ) //壁越えモータの角加速度rad/ss
    #define DISTANCE_WALL ( 0.180 )  //壁越えに必要な距離[m](機構が90度回転する)
#elif DRIVE_MODE == DRIVE_DUALWHEEL
    // 双輪キャスター関連
    #define PIN_CSB     ( 10 )    // turntableのPIN(CSB)
    #define RADIUS_R    ( 0.04 )    // wheel radius
    #define RADIUS_L    ( 0.04 )    // wheel radius
    #define W           ( 0.265 )    // tread
    #define GEARRATIO   ( 5.5 )
    #define TT_RES4     ( 4096 )    // turntableの分解能
    #define _2RES_PI    ( 2 * 2048 / 3.141592 ) // 駆動輪の角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数  
    #define _2RES_PI_T  ( 2 * 500 / 3.141592 ) //  ターンテーブルの角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数
#elif DRIVE_MODE == DRIVE_MECHANUM_8WHEELS
    // 8輪メカナム関連
    #define MECHANUM_RES			( 500 ) //500pulses
    #define MECHANUM_HANKEI		( 0.05 ) // 駆動輪の半径
    #define MECHANUM_HANKEI_A_OUTSIDE	( 0.15561 ) // 外側のホイール a[m]
    #define MECHANUM_HANKEI_B_OUTSIDE	( 0.26023 ) // 外側のホイール b[m]
    #define MECHANUM_HANKEI_A_INSIDE	( MECHANUM_HANKEI_A_OUTSIDE ) // 内側のホイール a[m] (外側と等しい)
    #define MECHANUM_HANKEI_B_INSIDE	( 0.26023 ) // 内側のホイール b[m]
    #define MECHANUM_HANKEI_A	( MECHANUM_HANKEI_A_OUTSIDE ) // 4輪接地のa[m] (内側と外側どちらとも等しい)
    #define MECHANUM_HANKEI_B	(( MECHANUM_HANKEI_B_OUTSIDE + MECHANUM_HANKEI_B_INSIDE ) / 2.0 ) // 4輪接地のb[m]
    #define _4RES_2PI_WHEEL ( 4 * 3 / ( 2.0*PI_ ) ) // 駆動輪の角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数(3ppr)
    #define _4RES_2PI_WALL ( 4 * 3 / ( 2.0*PI_ ) ) // 壁越え機構の角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数(3ppr)
    #define GEARRATIO_WHEEL ( 1.0 ) // 駆動輪のギヤ比
    #define GEARRATIO_WALL ( 1.0 )// 壁越え機構ギヤ比
    #define MAXVEL ( 1.0 )
    #define MAXOMEGA ( PI_ )
#elif DRIVE_MODE == DRIVE_OMNI3WHEEL
    #define _2RES_PI    ( 2 * 3 / 3.141592 ) // 駆動輪の角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数(3ppr)
    #define WHEEL_R		( 0.019 )  
    #define DIST2WHEEL  ( 0.120 )   
    #define GEARRATIO   ( 51.45)   
    #define COS_PI_6    ( 0.86602540378 )
    #define SIN_PI_6    ( 0.5 )
#elif DRIVE_MODE == DRIVE_OMNI4WHEEL //学生ロボコン2020　TR
    #define _2RES_PI    ( 2 * 200 / 3.141592 ) // 駆動輪の角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数 (1000ppr)
    #define WHEEL_R		( 0.0635 )
    #define DIST2WHEEL  ( 0.42 )
    #define GEARRATIO   ( 1.0 )
    #define SINCOS_PI_4 ( 0.70710678118 )
    //#define WHEELXY_R   (0.02) // meter
    #define MAX_VEL     (1.0)
    #define MAX_OMEGA   (2.094395102393)
    #define HIGH_VELOCITY (2.0)
    #define LOW_VELOCITY  (0.3)

#endif

// RoboClaw関連
#define ADR_MD_WHEE_2   ( 131 ) //Wheel2
#define ADR_MD_WHEE_1   ( 128 ) //Wheel1
#define ADR_MD_WHEE_4   ( 129 ) //Wheel4
#define ADR_MD_WHEE_3   ( 130 ) //Wheel3

#endif

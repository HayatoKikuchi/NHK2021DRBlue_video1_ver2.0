// ゲームコントローラのジョイスティックデータから，
// 指令速度を計算するクラス
// 作成日：2019年12月30日
// 作成者：上野祐樹
// 編集者：菊池隼(2021/02/10)

#ifndef MANUALCONTROL_h
#define MANUALCONTROL_h

#include "define.h"
#include "PIDclass.h"

#define JOY_DEADBAND    ( 20 )
#define JOY_MAXVEL      ( 1.0 )
#define JOY_MAXANGVEL   ( PI_ / 2.0 )

#define JOYCONMODE  1
#define JOYCONPID   2
#define POSITIONPID 3

class ManualControl{
public:
    /*********** 変数宣言 ***********/

    double refKakudo;
    double tmpPx, tmpPy;

    /*********** 関数宣言 ***********/
    ManualControl(PID *_pid);
    
    coords getGlobalVel(unsigned int JoyX, unsigned int JoyY, unsigned int JoyZ);
    coords getLocalVel(double refVx, double refVy, double refVz, double roboAngle,int mode);
    coords getVel_max(double vel_x, double vel_y, double vel_z);
    coords_4 getCmd(double refVx, double refVy, double refVz);
    double updatePosiPID(double conZ_or_position,double maxomega, double roboAngle, int mode); // タイマー割込みで使用
    void setRefAngle(double angle); //目標角度を設定，angle[deg]
    //coords_4 getMechanumWheelAccel(double vel_x, double vel_y, double vel_z,double robotAngle,double mode,double robotAccel);

    double robot_vel_x;
    double robot_vel_y;
    double robot_vel;
    double refAngle;

private:
    PID *pid;

    double posiZ_cmd;

    int path_num;
    int mode;
    int max_pathnum;

    double conv_length;
    double conv_tnum;

    bool mode_changed;
    bool init_done;

    double tan, per, rot;
};

#endif
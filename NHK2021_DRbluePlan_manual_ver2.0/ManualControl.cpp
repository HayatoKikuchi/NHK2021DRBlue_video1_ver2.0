// ゲームコントローラのジョイスティックデータから，
// 指令速度を計算するクラス
// 作成日：2019年12月30日
// 作成者：上野祐樹

#include "ManualControl.h"
#include "Filter.h"
#include "PIDclass.h"

Filter velX_filter(INT_TIME);
Filter velY_filter(INT_TIME);
Filter velZ_filter(INT_TIME);

ManualControl::ManualControl(PID *_pid){
  pid = _pid;
  posiZ_cmd = 0.0;
  velX_filter.setLowPassPara(0.10, 0.0);//Tと初期値を設定
  velY_filter.setLowPassPara(0.10, 0.0);//Tと初期値を設定
  velZ_filter.setLowPassPara(0.10, 0.0);//Tと初期値を設定
}

coords ManualControl::getGlobalVel(unsigned int JoyX, unsigned int JoyY, unsigned int JoyZ){
  int joyraw;
  coords rawV, refV;

  // ジョイスティックから指令速度を計算する
  joyraw = (int)JoyX - 127; 
  if(abs(joyraw) >= JOY_DEADBAND){
    if(joyraw >= 0){
      joyraw -= JOY_DEADBAND;
    }else{
      joyraw += JOY_DEADBAND;
    }
    rawV.x = -(joyraw)/(127.0 - (double)JOY_DEADBAND) * JOY_MAXVEL;
  }else{
    rawV.x = 0.0;
  }
  refV.x = velX_filter.LowPassFilter(rawV.x);

  joyraw = (int)JoyY - 127; 
  if(abs(joyraw) >= JOY_DEADBAND){
    if(joyraw >= 0){
      joyraw -= JOY_DEADBAND;
    }else{
      joyraw += JOY_DEADBAND;
    }
    rawV.y = -(joyraw)/(127.0 - (double)JOY_DEADBAND) * JOY_MAXVEL;
  }else{
    rawV.y = 0.0;
  }
  refV.y = velY_filter.LowPassFilter(rawV.y);

  joyraw = (int)JoyZ - 127; 
  if(abs(joyraw) >= JOY_DEADBAND){
    if(joyraw >= 0){
      joyraw -= JOY_DEADBAND;
    }else{
      joyraw += JOY_DEADBAND;
    }
    rawV.z = -(joyraw)/(127.0 - (double)JOY_DEADBAND) * JOY_MAXANGVEL;
  }else{
    rawV.z = 0.0;
  }
  refV.z = velZ_filter.LowPassFilter(rawV.z);
  //refV.z = rawV.z;

  robot_vel_x = refV.x;
  robot_vel_y = refV.y;

  return refV;
}

double ManualControl::updatePosiPID(double conZ_or_position,double maxomega, double roboAngle, int mode)
{
  switch (mode)
  {
  case JOYCONMODE:
    posiZ_cmd = conZ_or_position;
    break;
  
  case JOYCONPID:
    refAngle += conZ_or_position * 0.01;
    posiZ_cmd = pid->getCmd(refAngle,roboAngle,maxomega);
    break;
  
  case POSITIONPID:
    posiZ_cmd = pid->getCmd(refAngle,roboAngle,maxomega);
    break;

  default:
    break;
  }
  return refAngle;
}

void ManualControl::setRefAngle(double angle)
{
  refAngle = angle / 360.0 * 2.0 * PI_ ;
}

coords ManualControl::getLocalVel(double vel_x, double vel_y, double vel_z, double roboAngle,int mode){
  coords refVel;
  refVel.x = +vel_x*cos(roboAngle) + vel_y*sin(roboAngle);
  refVel.y = -vel_x*sin(roboAngle) + vel_y*cos(roboAngle);
  switch (mode)
  {
  case JOYCONMODE:
    refVel.z = vel_z;
    break;
  case JOYCONPID:
    refVel.z = posiZ_cmd;
    break;
  case POSITIONPID:
    refVel.z = posiZ_cmd;
    break;
  default:
    break;
  }

  return refVel;
}

coords ManualControl::getVel_max(double vel_x, double vel_y, double vel_z){
  coords refV;
  double vel;
  vel = sqrt(vel_x*vel_x + vel_y*vel_y);
  if(vel <= 0.0){
    refV.x = vel_x;
    refV.y = vel_y;
  }else if(fabs(vel_x) > fabs(vel_y)){
    refV.x = vel_x*fabs(vel_x) / vel;
    refV.y = vel_y*fabs(vel_x) / vel;
  }else if(fabs(vel_x) <= vel_y){
    refV.x = vel_x*fabs(vel_y) / vel;
    refV.y = vel_y*fabs(vel_y) / vel;
  }
  refV.z = vel_z;

  // robot_vel_x = refV.x;
  // robot_vel_y = refV.y;
  // robot_vel   = sqrt(refV.x*refV.x + refV.y*refV.y);

  return refV;
}

coords_4 ManualControl::getCmd(double refVx, double refVy, double refVz){

  //**ロボットの正面をx軸の正方向として第n象限のモータをMnとする**//

  /* 
    *      X
    *      ^
    *   M1 | M4
    * Y<---+----
    *   M2 | M3
    *      |
  */

  coords_4 refOmega;
  refOmega.i   = (refVx - refVy - refVz * (MECHANUM_A + MECHANUM_B)) / MECANUM_HANKEI * GEARRATIO_WHEEL;
  refOmega.ii  = (refVx + refVy - refVz * (MECHANUM_A + MECHANUM_B)) / MECANUM_HANKEI * GEARRATIO_WHEEL;
  refOmega.iii = (refVx - refVy + refVz * (MECHANUM_A + MECHANUM_B)) / MECANUM_HANKEI * GEARRATIO_WHEEL;
  refOmega.iv  = (refVx + refVy + refVz * (MECHANUM_A + MECHANUM_B)) / MECANUM_HANKEI * GEARRATIO_WHEEL;

  coords_4 VelCmd;
  VelCmd.i   = refOmega.i   * _4RES_2PI_WHEEL;
  VelCmd.ii  = refOmega.ii  * _4RES_2PI_WHEEL;
  VelCmd.iii = refOmega.iii * _4RES_2PI_WHEEL;
  VelCmd.iv  = refOmega.iv  * _4RES_2PI_WHEEL;

  return VelCmd;
}
/*
coords_4 ManualControl::getMechanumWheelAccel(double accel_x, double accel_y, double accel_z,double robotAngle,double mode,double robotAccel)
{
  coords GlobalAccel = getGlobalVel(accel_x,accel_y,accel_z); //速度の変換関数を加速度の変換に使用
  coords LocalAccel = getLocalVel(GlobalAccel.x,GlobalAccel.y,GlobalAccel.z,robotAngle,mode); //速度の変換関数を加速度の変換に使用
  coords_4 WheelAccel = getCmd(LocalAccel.x,LocalAccel.y,LocalAccel.z); //速度の変換関数を加速度の変換に使用
  return WheelAccel;
}*/
#ifndef __Timer_H_
#define  __Timer_H_
/* 좌측 모터: M1B +, M1A-*/
/* 우측 모터: M2B +, M2A-*/
#include "avr.h"

/*제어기를 위한 자료형*/
typedef struct
{
  /*제어기 이득 값*/
  float Kp;
  float Ki;
  
  /*저장해야할 값들*/
  float prevError;
  float integrator;
  float anti;
  
  /*제어기 출력*/
  float out;
}pid;

void timer2Set(); //타이머2 작동
void timer1Set();  //타이머1 작동

static int PI_controller(float goal, float real, pid *motor);  //PI제어기
static float encoderToAngular(int);  //엔코더 값을 각속도로 변환

#endif
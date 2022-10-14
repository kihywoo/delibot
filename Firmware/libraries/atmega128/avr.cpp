#include "avr.h"

extern volatile float angularVelocityGoal[2]; //목표 각속도 변수
extern volatile float angularVelocity[2]; //실제 각속도를 위한 변수
extern pid controllerParameter[2];  //제어기 변수
extern volatile int encoderReader[2]; //엔코더 펄스 변수

/*로봇 구동을 위한 모든 기능 ON*/
void MCU::init()
{
  exiSet();
  robotPort();
  timer2Set();
  timer1Set();
  setUSART(19200);
  sei();
}

/*모터 제어 함수*/
void MCU::controlMotor(float * value)
{
  angularVelocityGoal[LEFT] = (value[LINEAR]-(0.5f*value[ANGULAR]*WHEELBASE))/RADIUS; //이동로봇 기구학
  angularVelocityGoal[RIGHT] = (value[LINEAR]+(0.5f*value[ANGULAR]*WHEELBASE))/RADIUS;
}

/*제어기 이득을 화면으로 보기 위한 함수*/
void MCU::readcontrolGain(char arr[]) 
{
  //Kp = LEFT.Kp;
  //Ki = LEFT.Ki;
  
  char tmp1[10];
  char tmp2[10];
  dtostrf(controllerParameter[LEFT].Kp,3,1,tmp1);
  dtostrf(controllerParameter[LEFT].Ki,3,1,tmp2);
  sprintf(arr, "%s   %s", tmp1, tmp2);
}

/*실제 각속도를 화면으로 보기 위한 함수*/
void MCU::readAngularVelocity(char arr[]) 
{
  char tmp1[10];
  char tmp2[10];
  dtostrf(angularVelocity[LEFT],4,2,tmp1);
  dtostrf(angularVelocityGoal[RIGHT],4,2,tmp2);
  sprintf(arr, "%s   %s", tmp1, tmp2);
}

/*제어기 이득을 고치기 위한 함수*/
void MCU::modifyControlGain(float Kp, float Ki)
{
  controllerParameter[LEFT].Kp = Kp;
  controllerParameter[RIGHT].Kp = Kp;

  controllerParameter[LEFT].Ki = Ki;
  controllerParameter[RIGHT].Ki = Ki;
}

/*엔코더 펄스를 읽기 위한*/
void MCU::readEncoderTick(int & leftTick, int & rightTick)
{
    leftTick = encoderReader[LEFT];
    rightTick = encoderReader[RIGHT];
}

/*엔코더 펄스를 초기화 하기 위한*/
void MCU::clearEncoderTick(void)
{
    encoderReader[LEFT]=0;
    encoderReader[RIGHT]=0;
}
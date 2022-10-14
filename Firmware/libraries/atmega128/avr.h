#ifndef AVR_H_
#define AVR_H_

#include <avr/io.h> //레지스터 사용 위해 선언
#include <avr/interrupt.h>  //ISR사용 위해 선언
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include "PORT.h"
#include "Timer.h"
#include "EXI.h"
#include "USART.h"


/*모터 관련 파라미터*/
#define LINEAR      0
#define ANGULAR     1

#define LEFT 0
#define RIGHT 1

#define  RADIUS 0.075f      //바퀴의 반지름(m)
#define  WHEELBASE  0.625f  //휠 베이스 간격(m)

#define  PERIOD 750  //TOP값 설정, PWM 주기 결정

#define  SAMPLINGTIME 0.01f  //제어 및 샘플링 주기
#define  MAX 23.0f  //포화의 최대 값

#define  TICK2RAD 1.59e-3  //엔코더 1바퀴시 발생하는 펄스 수

class MCU{
  public:
  void init();  //로봇 구동을 위한 모드 기능 활성화
  void controlMotor(float * value); //주기마다 모터를 구동하기 위한 변수, 기구학 사용
  void readcontrolGain(char arr[]); //제어기 이득을 얻기 위한 함수
  void readAngularVelocity(char arr[]) ;  //실제 각속도를 읽기 위한 함수
  void modifyControlGain(float Kp, float Ki); //제어기 이득을 변경하기 위한 함수
  void readEncoderTick(int & leftTick, int & rightTick);  //엔코더의 펄스 수를 읽기 위한 함수
  void clearEncoderTick(void);  //엔코더 펄스를 초기화 하는 함수

  private:
};

#endif /* AVR_H_ */
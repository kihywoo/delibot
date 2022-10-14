#include "PORT.h"

void robotPort()
{
  DDRD = 0x00;  //외부인터럽트
  
  DDRA |= (1<<DDA1); //모터 방향 핀 출력 설정
  DDRF |= (1<<DDF1); 
  
  DDRB |= (1<<DDB5) | (1<<DDB6); //모터 PWM 핀 출력 설정
}
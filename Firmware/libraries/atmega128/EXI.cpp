//왼쪽 모터의 A채널 INT0, B채널 INT2
//오른쪽 모터의 A채널 INT1, B채널 INT3
//모터가 CW방향으로 회전시 B채널이 먼저 들어옴을 가정
//로봇 전진은 오른쪽 모터 CW, 왼쪽 모터 CCW
//2체배 사용

#include "EXI.h"  //헤더파일과 연결

volatile int encoderCounter[2];  //엔코더 펄스 수를 세기 위한 변수(제어기)
volatile int encoderReader[2];  //엔코더 펄스 수를 세기 위한 변수(조인트 값)

ISR(INT0_vect)  //왼쪽 모터 A채널 인터럽트
{
  if(EICRA&(1<<ISC00)) //rising edge에서 인터럽트가 발생한 것이면
  {
    if(!(PIND&(1<<PIND2)))
    { 
      encoderCounter[LEFT]++; 
      encoderReader[LEFT]++; //B상이 low이면 CCW, 왼쪽 모터 전진
    }
    else 
    {
      encoderCounter[LEFT]--;  
      encoderReader[LEFT]--;//B상이 high이면 CW, 왼쪽 모터 후진
    }
  }
  else  //falling edge에서 인터럽트가 발생한 것이면
  {
    if((PIND&(1<<PIND2))) 
    {
      encoderCounter[LEFT]++;  
      encoderReader[LEFT]++;//B상이 high이면 CCW, 왼쪽 모터 전진
    }
    else 
    {
      encoderCounter[LEFT]--;  
      encoderReader[LEFT]--;//B상이 low이면 CW, 왼쪽 모터 후진
    }
  }
  EICRA ^= (1<<ISC00);  //인터럽트 발생 마다 rising, falling edge에서 인터럽트를 받을건지 바꿔줌
}

ISR(INT1_vect)  //오른쪽 모터 A채널 인터럽트
{
  if(EICRA&(1<<ISC10)) //rising edge에서 인터럽트가 발생한 것이면
  {
    if(PIND&(1<<PIND3)) 
    {
      encoderCounter[RIGHT]++; 
      encoderReader[RIGHT]++;//B상이 high이면 CW, 오른쪽 모터 전진
    }
    else 
    {
      encoderCounter[RIGHT]--;  
      encoderReader[RIGHT]--; //B상이 low이면 CCW, 오른쪽 모터 후진
    }
  }
  else
  {
    if(!(PIND&(1<<PIND3))) 
    {
      encoderCounter[RIGHT]++; 
      encoderReader[RIGHT]++;//B상이 low이면 CW, 오른쪽 모터 전진
    }
    else 
    {
      encoderCounter[RIGHT]--; 
      encoderReader[RIGHT]--;//B상이 high이면 CCW, 오른쪽 모터 후진
    }
  }
  
  EICRA ^= (1<<ISC10);  //인터럽트 발생 마다 rising, falling edge에서 인터럽트를 받을건지 바꿔줌
}

void exiSet()  //외부 인터럽트 레지스터 설정 함수
{
  EIMSK = (0<<INT3)|(1<<INT1)|(0<<INT2)|(1<<INT0);   //외부인터럽트 0,1 허용
  EICRA = (1<<ISC31)|(1<<ISC30)|(1<<ISC21)|(1<<ISC20)|(1<<ISC11)|(1<<ISC10)|(1<<ISC01)|(1<<ISC00);  //모든 입력을 상승 엣지에서 받음
}
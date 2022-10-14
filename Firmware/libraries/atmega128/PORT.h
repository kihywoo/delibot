//오른쪽 모터 방향 제어 PORTA2, 왼쪽 모터 방향 제어 PORTA1
//오른쪽 모터 속도 제어 PORTB5, 왼쪽 모터 속도 제어 PORTB6
//PORTD0~3은 외부 인터럽트 사용
//로봇 전진은 오른쪽 모터 CW, 왼쪽 모터 CCW
#ifndef PORT_H_
#define PORT_H_

#include "avr.h"

#define RIGHT_MOTER_CCW() (PORTF)&=(~((1)<<(PORTF1)))  //오른쪽 모터 CW 방향 회전
#define RIGHT_MOTER_CW() (PORTF)|=(((1)<<(PORTF1)))  //오른쪽 모터 CCW 방향 회전

#define LEFT_MOTER_CCW() (PORTA)&=(~((1)<<(PORTA1)))   //왼쪽 모터 CW 방향 회전
#define LEFT_MOTER_CW() (PORTA)|=(((1)<<(PORTA1)))   //왼쪽 모터 CCW 방향 회전

void robotPort();   //로봇을 구동하기 위한 모든 GPIO 설정

#endif /* PORT_H_ */
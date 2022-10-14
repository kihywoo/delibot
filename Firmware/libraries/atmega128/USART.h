#ifndef USART_h_
#define  USART_h_

#include "avr.h"

#define F_CPU 16000000UL
#define BUF_SIZE 50  //입력 받을 문자열 size를 상수로 선언

void setUSART(int);  //USART 통신 준비하는 함수
void serialWriteln(char*);  //데이터를 보내는 함수
void serialWritelnNoInterrup(char* buf);

#endif
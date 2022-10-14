#include "Timer.h"

extern volatile int encoderCounter[2];  //엔코더 출력

volatile char count; //타이머 2, 제어 변수

volatile float angularVelocity[2];  //실제 출력 각속도
volatile float angularVelocityGoal[2];  //목표 출력 각속도
pid controllerParameter[2] = {{.Kp=1.0, .Ki=40.0},{.Kp=1.0, .Ki=40.0}}; //각 모터의 제어기 파라미터 변수

ISR(TIMER2_OVF_vect)  //1ms마다 발생
{ 
  count++;
  
  if(count==10)  //0.01초 마다 제어 및 샘플링
  {
    angularVelocity[LEFT]=encoderToAngular(encoderCounter[LEFT]);   // 엔코더를 이용한 왼쪽 모터의 실제 각속도(rad/sec)
    angularVelocity[RIGHT]=encoderToAngular(encoderCounter[RIGHT]); // 엔코더를 이용한 오른쪽 모터의 실제 각속도(rad/sec)
    
    OCR1A=PI_controller(angularVelocityGoal[LEFT], angularVelocity[LEFT], &controllerParameter[LEFT]);  //제어량 반환
    OCR1B=PI_controller(angularVelocityGoal[RIGHT], angularVelocity[RIGHT], &controllerParameter[RIGHT]); //제어량 반환

    encoderCounter[LEFT]=0;   //0.01초마다 엔코더값 초기화(샘플링 타임)
    encoderCounter[RIGHT]=0;  

    count=0;
  }
  
  TCNT2=6;  //타이머 초기화
}

void timer2Set()
{
  TCCR2 = (0<<WGM21) | (0<<WGM20) | (0<<CS22)| (1<<CS21)| (1<<CS20);  //일반 모드, 64분주
  TIMSK |= (1<<TOIE2) | (0<<OCIE0);  //오버플로우 인터럽트 허용
  TCNT2=6;  //1ms를 만들기 위해
}

void timer1Set()
{ 
  TCCR1A = (1<<WGM11) | (0<<WGM10) | (1<<COM1A1)| (0<<COM1A0)| (1<<COM1B1)| (0<<COM1B0);  // PC PWM 모드(TOP=ICR1)
  TCCR1B = (1<<WGM13) | (0<<WGM12) | (0<<CS12)| (0<<CS11)| (1<<CS10);  //No prescaling
  //TIMSK = (0<<TOIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (0<<TICIE1);  //모든 인터럽트 허용하지 않음
  
  ICR1 = PERIOD;   //TOP값을 PERIOD값으로 설정
} 

 static int PI_controller(float setpoint, float measurement, pid *motor)
 {
  float error = setpoint - measurement; //현재 오차
  
  float proportional = motor->Kp * error; //비례항
  
  motor->integrator += motor->Ki  * ((0.5 * (error + motor->prevError))- motor->anti)* SAMPLINGTIME; //적분항, 사다리꼴 수치 적분 사용
  
  motor->prevError = error; //이전 오차= 현재 오차
  
  motor->out = proportional + motor->integrator;  //제어기 출력
  
  
  /*제어기 출력 포화 제한*/
  if(motor->out>MAX)
  {
    motor->anti = motor->out - MAX;		//제어 출력 포화 발생시 안티 와인드업 발생
    motor->out = MAX;
  }
  else if(motor->out<-MAX)
  {
    motor->anti = motor->out + MAX;
    motor->out = -MAX;
  }
  else
  {
    motor->anti = 0;
  }
  
  /*제어기의 출력에 따른 순간적인 모터 방향 변화 방지*/
  if((motor->out*setpoint)<0)
  {
    if(setpoint>0) motor->out = 1;
    else motor->out = -1;
  } 
  
  
  /*제어기 출력에 따른 모터 방향 변화*/
  if(motor->out<0)
  {
    if(motor==&controllerParameter[LEFT])
    {
      LEFT_MOTER_CW();
    }
    else
    {
      RIGHT_MOTER_CCW();
    }
    motor->out = -motor->out;
  }
  else
  {
    if(motor==&controllerParameter[LEFT])
    {
      LEFT_MOTER_CCW();
    }
    else
    {
      RIGHT_MOTER_CW();
    }
  }

  if(setpoint==0 && measurement ==0)
  {
    motor->integrator = 0;
  }
  
  return ((motor->out/24.0f)*PERIOD); //제어기 출력 반환
 }

static float encoderToAngular(int encoder) //엔코더값을 각속도로 변환
{
  float angularVelocity=0.0f;
  
  angularVelocity = (float)(encoder*TICK2RAD)/(SAMPLINGTIME);  // 2파이 rad:3952pulse = ?rad : 엔코더 실제 펄스수 --> ?rad = (엔코더*2파이)/3952 --> 각속도 --> ?rad/샘플시간 
   
  return angularVelocity;  //실제 각속도 값 반환
}   

#include "USART.h" //헤더 파일과 연결

static char txBuf[BUF_SIZE];  //전송할 데이터를 저장하는 버퍼
char rxBuf[BUF_SIZE];  //전송할 데이터를 저장하는 버퍼
static volatile int txIndex=0;  //버퍼의 인덱스에 접근하기 위한 변수
static volatile int rxIndex=0;

ISR(USART0_TX_vect)  //송신이 완료되면 발생하는 인터럽트
{
	if(txBuf[txIndex] != '\0')  //txBuf에 널 문자가 나올때 까지
	{
		UDR0 = txBuf[txIndex++];  //UDR0레지스터에 txBuf값을 저장하여 송신을 함
		delay(1);
	}
}

ISR(USART0_RX_vect)  //수신완료 인터럽트
{
	rxBuf[rxIndex] = UDR0;	//수신 데이터를 수신 버퍼에 저장
	
	if(rxBuf[rxIndex++]=='\0')
	{
		rxIndex=0;	//NULL을 받으면 NULL을 저장
	}
}

void setUSART(int baud)
{
	UCSR0B =(1<<RXCIE0) | (0<<TXCIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02); //송수신 인터럽트 허용, 송/수신 허용, 8비트 데이터 전송
	UCSR0C = (0<<UMSEL0) | (1<<UCSZ01) | (1<<UCSZ00);  //비동기 모드 사용, 8비트 데이터 전송
	
	int ubrrValue = (((F_CPU / (baud * 16UL))) - 1);
	//UBRR0H=(uint8_t)(ubrrValue>>8);
	//UBRR0L=(uint8_t)ubrrValue;  //9600bps 사용
	UBRR0H=0;
	UBRR0L=51;  //9600bps 사용
}

void serialWriteln(char* buf)
{
	int i=0;
	txIndex=0;  //가장 첫 인덱스에 접근하기 위해 초기화
	
	while(buf[i]!='\0')  //buf에 저장된 모든 데이터를 txBuf에 저장. 이때, 널 문자 제외
	{
		txBuf[i]=buf[i];
		i++;
	}
	
	txBuf[i++]='\n';  //모든 문자를 저장한 후 바로 다음 문자에 개행 문자 저장
	txBuf[i]='\0';  //그 다음엔 널 문자 저장
	
	UDR0 = txBuf[txIndex++];  //UDR0레지스터에 txBuf에 저장된 첫번째 문자를 저장하고 인덱스를 증가
	delay(1);
}

void serialWritelnNoInterrup(char* buf)
{
	int i=0;
	
	while(buf[i]!='\0')  //buf에 저장된 모든 데이터를 txBuf에 저장. 이때, 널 문자 제외
	{
		while(!(UCSR0A&0x20));
		UDR0 = buf[i];
		i++;
	}

	while(!(UCSR0A&0x20));
	UDR0 = '\n';
}
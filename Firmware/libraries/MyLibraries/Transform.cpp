#include "Arduino.h"
#include "Transform.h"
#include <math.h>

extern char instr[20];
double Coordinate_X, Coordinate_Y, Coordinate_Z, Angle_A;     // Angle_A = degree
int temp;

void reset_arr(char *instr, int len)              // 문자열을 초기화하기 위한 함수
{
	int i;
	
	for (i=0; i<len; i++)
	{
		instr[i] = '\0';
	}
}

void Transform (char *InstrPacket)             // PC에서 송신한 ASCII 코드 data를 정수형으로 변환
{
	int i = 0;
	
	if (InstrPacket[4] != 32 || InstrPacket[1] > 53 || InstrPacket[6] > 53 || CHECK)           // 300 이상의 좌표는 제한
	{
		//Serial.println("Unavailable Coordinates");
		return ;
	}

	for (i=1; i<4; i++)                                          // 배열의 첫 번째 요소는 부호, 나머지 2,3,4번 요소는 값을 나타냄
	{
		temp += (round(pow(10,3-i))*(InstrPacket[i]-48));
	}
	if (InstrPacket[0] == '-')                                   // 배열의 첫 번째 요소가 '-'이면 해당 값을 음수로 저장
	{
		Coordinate_X = -temp;
		Coordinate_X /= 10;
	}
	else
	{
		Coordinate_X = temp;
		Coordinate_X /= 10;
	}
	//Serial.print("x = ");
	//Serial.println(Coordinate_X);
	//delay(100);
	temp = 0;
	
	for (i=6; i<9; i++)                                          // 배열의 5번째 요소는 SP, 6번째 요소는 부호, 그 뒤의 요소는 값을 나타냄
	{
		temp += (round(pow(10,8-i))*(InstrPacket[i]-48));
	}
	if (InstrPacket[5] == '-')                                   // 배열의 6번째 요소가 '-'이면 해당 값을 음수로 저장
	{
		Coordinate_Y = -temp;
		Coordinate_Y /= 10;
	}
	else
	{
		Coordinate_Y = temp;
		Coordinate_Y /= 10;
	}
	//Serial.print("y = ");
	//Serial.println(Coordinate_Y);
	//delay(100);
	temp = 0;
}

void Transform_Z (char *InstrPacket)
{
    int i = 0;

    if ((InstrPacket[0] != '0') || InstrPacket[1] > 50 || InstrPacket[2] > 53 || InstrPacket[3] > 53)           //  25.5cm 이상의 높이는 제한
    {
        //Serial.println("Unavailable Coordinates");
        return ;
    }

    for (i=1; i<4; i++)                                          // 배열의 첫 번째 요소는 부호, 나머지 2,3,4번 요소는 값을 나타냄
    {
        temp += (round(pow(10, 3 - i)) * (InstrPacket[i] - 48));
    }

    Coordinate_Z = temp;
    Coordinate_Z /= 10;

    //Serial.print("z = ");
    //Serial.println(Coordinate_Z);
    //delay(100);
    temp = 0;
}

void Transform_A (char* InstrPacket)
{
    int i = 0;

    if (InstrPacket[1] > 51 || InstrPacket[2] > 57 || InstrPacket[3] > 57)
    {
        //Serial.println("Unavailable Angle");
        return ;
    }

    for (i=1; i<4; i++)                                          // 배열의 첫 번째 요소는 부호, 나머지 2,3,4번 요소는 값을 나타냄
    {
        temp += (round(pow(10,3-i))*(InstrPacket[i]-48));
    }
    if (InstrPacket[0] == '-')                                   // 배열의 첫 번째 요소가 '-'이면 해당 값을 음수로 저장
    {
        Angle_A = -temp;
    }
    else
    {
        Angle_A = temp;
    }
    //Serial.print("A = ");
    //Serial.println(Angle_A);
    //delay(100);
    temp = 0;
}

int CheckCoordinates (double X, double Y)
{
	if (pow(X, 2.0) + pow(Y, 2.0) > 1700 || pow(X, 2.0) + pow(Y, 2.0) < 49)                  // 반경이 7cm 미만 34cm 초과인 범위는 이동 불가
	{
		//Serial.println("Unavailable Coordinates");
		return 0;
	}
	else
	{
		return 1;
	}
}

int CheckCoordinate_Z (double Z)
{
    if (Z > 25.50)
    {
        //Serial.println("Unavailable Coordinate_Z");
        return 0;
    }
    else
        return 1;
}

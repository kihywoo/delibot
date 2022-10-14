#include "Arduino.h"
#include "InverseKinematics.h"
#include <math.h>

double C2;
double theta1, theta2;
double pre_theta1, pre_theta2;
int Theta1ToStep, Theta2ToStep, zToStep, aToStep;
extern double Coordinate_X, Coordinate_Y, Coordinate_Z;

void InverseKinematics (double X, double Y)
{
	C2 = ((double)(pow(X, 2.0) + pow(Y, 2.0) - pow(L1, 2.0) - pow(L2, 2.0))/(double)(2*L1*L2));        // MATLAB에서 계산한 cos(theta2) 수식
	//Serial.println(C2,5);
	if ((C2 < -1) || (C2 > 1))
	{
		//Serial.println("FAIL");
	}
	else if (C2 < 0)
	{
		if (abs(pre_theta2-THETA2_1) < abs(pre_theta2-THETA2_2))     // 이전 각도와 비교했을 때 움직여야하는 각도가 최소가 되도록 선택
		{
			theta2 = THETA2_1;
			pre_theta2 = theta2;
			//Serial.print("radian theta2 = ");
			//Serial.println(theta2,5);
		}
		else
		{
			theta2 = THETA2_2;
			pre_theta2 = theta2;
			//Serial.print("radian theta2 = ");
			//Serial.println(theta2,5);
		}
	}
	else if (C2 > 0)
	{
		if (abs(THETA2_3-pre_theta2) < abs(THETA2_4-pre_theta2))     // 이전 각도와 비교했을 때 움직여야하는 각도가 최소가 되도록 선택
		{
			theta2 = THETA2_3;
			pre_theta2 = theta2;
			//Serial.print("radian theta2 = ");
            //Serial.println(theta2,5);
		}
		else
		{
			theta2 = THETA2_4;
			pre_theta2 = theta2;
			//Serial.print("radian theta2 = ");
            //Serial.println(theta2,5);
		}
	}
	else if (C2 == 0)
	{
		if (sqrt(1-round(pow(C2, 2.0))) == 0)           // atan(y/x)에서 x,y 둘다 0이면 정의되지 않음
		{
			//Serial.println("Unavailable Angle");
		}
		
		else if (pre_theta2 <= 0)                  // 이전 theta2가 0보다 작은 경우, -90도가 더 가까움
		{
			theta2 = -HALF_PI;
			pre_theta2 = theta2;
			//Serial.print("radian theta2 = ");
            //Serial.println(theta2,5);
		}
		else if (pre_theta2 > 0)            // 이전 theta2가 0보다 큰 경우, 90도가 더 가까움
		{
			theta2 = HALF_PI;
			pre_theta2 = theta2;
			//Serial.print("radian theta2 = ");
            //Serial.println(theta2,5);
		}
	}
	
	theta1 = (atan2(Y,X) - atan((L2*sin(theta2))/(L1+L2*C2)));
	//Serial.print("radian theta1 = ");
	//Serial.println(theta1,5);

	//Serial.print("degree theta1 = ");
	//Serial.println(degrees(theta1),5);

	//Serial.print("degree theta2 = ");
	//Serial.println(degrees(theta2),5);

	Theta1ToStep = round((degrees(theta1)/0.45)*20);
	Theta2ToStep = round((degrees(theta2)/0.45)*16);
	//zToStep = round((25.5-Coordinate_Z)*(1003.921));
}

void Coordinate_zToStep (double Z)
{
    zToStep = round((25.5-Z)*(1003.921));
}

void GripperToStep (double A)
{
    aToStep = round((A/0.45)*4.5);
}

void CheckGripperStep (void)
{
   if (theta2<0 && theta1>0)
   {
     GripperToStep(degrees(abs(theta2))-degrees(abs(theta1))-degrees(PI/2));
   }

   else if (theta2>0)
   {
     GripperToStep(-(degrees(abs(theta2))-degrees(abs(theta1))-degrees(PI/2)));
   }

   else if (theta2<0 && theta1<0)
   {
     GripperToStep(degrees(abs(theta2)+abs(theta1)-(PI/2)));
   }
}

#ifndef InverseKinematics_H_
#define InverseKinematics_H_

void InverseKinematics (double X, double Y);
void Coordinate_zToStep (double Z);
void GripperToStep (double A);
void CheckGripperStep (void);
#define L1 24
#define L2 15.5
#define THETA2_1 (double)(atan(sqrt(1-pow(C2, 2.0))/C2) +PI)
#define THETA2_2 (double)(atan(-sqrt(1-pow(C2, 2.0))/C2)-PI)
#define THETA2_3 (double)(atan(sqrt(1-pow(C2, 2.0))/C2))
#define THETA2_4 (double)(atan(-sqrt(1-pow(C2,2.0))/C2))
#define THETA1 (double)(atan2(Y,X)-atan2(L2*sin(theta2),(L1+L2*C2)))

#endif 

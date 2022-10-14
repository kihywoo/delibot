#ifndef Transform_H_
#define Transform_H_

#define CHECK (InstrPacket[1] > 57 || InstrPacket[2] > 57 || InstrPacket[3] > 57 || InstrPacket[6] > 57 || InstrPacket[7] > 57 || InstrPacket[8] > 57)
void reset_arr(char *instr, int len);
void Transform (char *InstrPacket);
void Transform_Z (char *InstrPacket);
void Transform_A (char *InstrPacket);
int CheckCoordinates (double X, double Y);
int CheckCoordinate_Z (double Z);

#endif

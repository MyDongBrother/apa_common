#include "Nav_math.h"

//计算方差和期望
void Math_UpdateEVar(float input, float *E, float *Var, unsigned short bufferSize)
{
    *E += (input - *E) / bufferSize;
    *Var = (bufferSize - 2) * (*Var) / (bufferSize - 1) +
           (input - *E) * (input - *E) / bufferSize;
}

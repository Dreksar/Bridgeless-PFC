#ifndef _CLECK_H
#define _CLECK_H

#include "main.h"

#include "math.h"

#define PI  3.1415926f


typedef struct
{
	//��Cleck�任
	float Valpha;
	float Vbeta;
	float Va;
	float Vb;
	float Vc;
	
	//Cleck�任
	float Ialpha;
	float Ibeta;
	float Ia;
	float Ib;
	float Ic;
	
	
}CLECK;
void anti_cleck(CLECK *Cleck);
void cleck(CLECK *Cleck);

#endif
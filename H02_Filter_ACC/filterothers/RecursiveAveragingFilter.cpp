#include <iostream>

#include "Filters.h"

using namespace std;

static double RAFvalueBuffer[N] = { 0.f };





double Filters ::fRAFFilter_Main()
{


	double a = this->a;

	double res = 0.f;
	static double j = 0;
	double s_sum = 0.f;
	static int i = 0;
	RAFvalueBuffer[i++] = a;

	if (i == N)
	{
		i = 0;
	}

	for (int count = 0; count < N; count++) {
		s_sum += RAFvalueBuffer[count];
	}
	++j;
	j = (j >= N ? N : j);
	if (j < N)
	{
		return a;
	}
	else
	{
		return  s_sum / N;
	}

}
#include <iostream>
#include <algorithm>
#include "Filters.h"




using namespace std;

static double MVAFvalueBuffer[N] = { 0.f };




double Filters:: fMVAFFilter_Main()
{
	
	 double a = this->a;

	double res = 0.f;
	static int j = 0;
	double s_sum = 0.f;
	static int i = 0;
	MVAFvalueBuffer[i++] = a;

	if (i == N)
	{
		i = 0;
	}
	++j;
	j = (j >= N ? N : j);
	sort(MVAFvalueBuffer, MVAFvalueBuffer + j-1);
	for (int count = 1; count < N-1; count++) {
		s_sum += MVAFvalueBuffer[count];
	}
	if (j < N)
	{
		return a;
	}
	else
	{
		return  s_sum / (N - 2);
	}
}
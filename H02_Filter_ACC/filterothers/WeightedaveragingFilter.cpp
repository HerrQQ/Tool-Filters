#include <iostream>
#include <algorithm>
#include <queue>
#include <vector>
#include "Filters.h"




using namespace std;

static queue <double >  WAFvalueBuffer;

static vector <int > WAFweightcof(N, 0);




double Filters::fWAFFilter_Main()
{

	double a = this->a;
	static int j = 0 ;
	double s_sum = 0.f;
	int sum_cof = 0;
	static int i = 0;
	WAFvalueBuffer.push(a);
	i++;
	if (i > N)
	{
		WAFvalueBuffer.pop();
		i = N;
	}

	queue <double > buffer_t(WAFvalueBuffer);
	for (int m = 0; m < i; m++)
	{
		WAFweightcof[m] = m + 1;
		sum_cof += (m + 1);
		s_sum += buffer_t.front()* WAFweightcof[m];
		buffer_t.pop();
	}
	++j;
	j = (j >= N ? N : j);

	if (j < N)
	{
		return a;
	}
	else
	{
		return  s_sum / sum_cof;
	}
}
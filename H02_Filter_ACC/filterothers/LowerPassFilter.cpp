#include <iostream>
#include <algorithm>
#include "Filters.h"

using namespace std;

void Filters:: fgetValue(double value)
{
	this->a=value;
}

double Filters::fLPFFilter_Main()
{
	double a_new=0.f;
	static double s_a_old = 0.f;

	 a_new = this->a;
	a_new = s_a_old * LPF_80 + (1.0 - LPF_80) * a_new ;
	s_a_old = a_new;
	return a_new;
}

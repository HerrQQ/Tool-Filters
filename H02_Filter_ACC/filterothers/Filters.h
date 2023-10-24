#pragma once

#include "testMain.h"

using namespace std;

class Filters

{
public:
	void fgetValue(double value);
	double fLPFFilter_Main();
	double fMVAFFilter_Main();
	double fRAFFilter_Main();
	double fWAFFilter_Main();

private:

	double a = 0, f;
};
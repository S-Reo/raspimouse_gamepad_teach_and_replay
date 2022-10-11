#include "Observation.h"
#include <iostream>
#include <string>
#include <cmath>

Observation::Observation()
{
}

Observation::Observation(int left_f,int left_s, int right_s, int right_f)
{
	setValues(left_f,left_s,right_s,right_f);
}

void Observation::setValues(int left_f,int left_s, int right_s, int right_f)
{
	lf = left_f > 0 ? left_f : 1;
	ls = left_s > 0 ? left_s : 1;
	rs = right_s > 0 ? right_s : 1;
	rf = right_f > 0 ? right_f : 1;

	/*
	log_lf = log10((double)lf);
	log_ls = log10((double)ls);
  log_rs = log10((double)rs);
	log_rf = log10((double)rf);
  */
  
  log_lf = 1+(pow(1.03,(float)lf*0.02)*0.1f);
  log_ls = 1+(pow(1.03,(float)ls*0.02)*0.1f);
  log_rs = 1+(pow(1.03,(float)rs*0.02)*0.1f);
  log_rf = 1+(pow(1.03,(float)rf*0.02)*0.1f);
  

  /* 
  log_lf = (double)lf*0.001;
  log_ls = (double)ls*0.001;
  log_rs = (double)rs*0.001;
  log_rf = (double)rf*0.001;
  */
}

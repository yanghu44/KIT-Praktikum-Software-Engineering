#include "PID_Regler.h"


PID_Regler::PID_Regler(double K_p, double K_i, double K_d, double max, double min, double soll, int i_window_length, double cycle_time) {
	this->K_p = K_p;
	this->K_i = K_i;
	this->K_d = K_d;
	this->max = max;
	this->min = min;
	this->i_window_length = int(i_window_length/cycle_time);
	this->cycle_time = cycle_time;
	sollwert = soll;
	//set_sollwert(soll);
}

PID_Regler::PID_Regler(double K_p, double K_i, double K_d, double max, double min, double soll, int i_window_length, double cycle_time, double sollwert_min, double sollwert_max) {
	this->K_p = K_p;
	this->K_i = K_i;
	this->K_d = K_d;
	this->max = max;
	this->min = min;
	this->i_window_length = int(i_window_length / cycle_time);
	this->cycle_time = cycle_time;
	this->sollwert_min = sollwert_min;
	this->sollwert_max = sollwert_max;
	set_sollwert(soll);
}


retVals PID_Regler::regel_algo(double istwert) {
	//num_calls++;

	double e = sollwert - istwert;

	double p = K_p * e;

	double integ = cycle_time * e;
	i_sum = i_sum + integ;
	i_queue.push(integ);
	while (i_queue.size() > i_window_length) {
		i_sum = i_sum - i_queue.front();
		i_queue.pop();
	}

	double i = K_i * i_sum;
	
	double d = K_d * (e - e_prev)/cycle_time;
	e_prev = e;
			
	double output = p + i + d;

	if (output > max) {
		output = max;
	}
	else if (output < min) {
		output = min;
	}
	

	return retVals{output, p, i, d };
}

void PID_Regler::set_sollwert(double soll) {
	sollwert = soll;
}

void PID_Regler::set_sollwert_max(double soll) {
	sollwert_max = soll;
	//Log("Neuer max sollwert im Regler: %lf \n", sollwert_max);
}

//changed

#include "Abstandsregler.h"


Abstandsregler::Abstandsregler(double cycle_time) : PID_Regler(0.05, 0.01, 0.05, 5.0, -5.0, 100.0, 2, cycle_time, 50.0, 200.0) {
     
     d_queue_past.push(0);
     d_queue_curr.push(0);
     
}



retVals Abstandsregler::abstandsRegelung(double dist, double Obj_v, double safety_dist_factor) {
    

    
    double soll_dist =  30 + Obj_v * safety_dist_factor / 4.0;         //original: 30 + v*2.5
    set_sollwert(soll_dist);

    retVals regler_values = regel_algo(dist);
    //Log("D-Anteil: %lf \n", regler_values.d_out);
    double reglerOutput = regler_values.sum_out;


    return regler_values;
  
    
};

retVals Abstandsregler::regel_algo(double istwert) {
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
    
    
    d_sum_curr = d_sum_curr + e;
    d_queue_curr.push(e);
    while (d_queue_curr.size() > 5) {
        d_sum_curr = d_sum_curr - d_queue_curr.front();
        d_queue_past.push(d_queue_curr.front());
        d_sum_past = d_sum_past + d_queue_curr.front();
        d_queue_curr.pop();
    }
    while (d_queue_past.size() > 5) {
        d_sum_past = d_sum_past - d_queue_past.front();
        d_queue_past.pop();
    }
    //Log("D anteil sum_curr: %lf , sum_past: %lf \n", d_sum_curr, d_sum_past);
    double diff = 0.0;
    if (d_queue_past.size() == 5) {
        diff = d_sum_curr / double(d_queue_curr.size()) - d_sum_past / double(d_queue_past.size());
    }
    
    //Log("D anteil diff: %lf, curr_size: %i , past_size: %i \n", diff, d_queue_curr.size(), d_queue_past.size());
    if (abs(diff) > 5) {
        diff = 0;
    }
    double d = K_d * diff / cycle_time;


    if (p > max) {
        p = max;
    }
    else if (p < min) {
        p = min;
    }
    if (i > max) {
        i = max;
    }
    else if (i < min) {
        i = min;
    }
    if (d > max) {
        d = max;
    }
    else if (d < min) {
        d = min;
    }
    double output = p + i + d;

    if (output > max) {
        output = max;
    }
    else if (output < min) {
        output = min;
    }


    return retVals{ output, p, i, d };
}

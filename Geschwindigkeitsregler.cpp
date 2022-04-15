#include "Geschwindigkeitsregler.h"


Geschwindigkeitsregler::Geschwindigkeitsregler(double limit, double cycle_time) : PID_Regler(0.2, 0.001, 0.5, 1.0, -1.0, limit, 5, cycle_time, 0.0, 37.0) {
    this->nextGas = 0;
}



retVals Geschwindigkeitsregler::geschwindigkeitsRegelung() {

    retVals regler_values =  regel_algo(Vehicle.v);
    double reglerOutput = regler_values.sum_out;
    //Log("v_regler Sollgeschwindigkeit: %lf \n", sollwert);
    nextGas = nextGas + reglerOutput * cycle_time;      //evtl faktor anpassen

    if (nextGas > 1) {
        nextGas = 1;
        VehicleControl.Gas = nextGas;
        VehicleControl.Brake = 0;
    }
    else if (nextGas < 0) {
        VehicleControl.Brake = -0.4 * nextGas;
        VehicleControl.Gas = 0;
        if (nextGas < -1) {
            nextGas = -1;
        }
    }
    else {
        VehicleControl.Gas = nextGas;
        VehicleControl.Brake = 0;
    }
    
    return regler_values;
}

void Geschwindigkeitsregler::set_sollwert(double soll) {
    if (soll > sollwert_max) {
        sollwert = sollwert_max;
    }
    else if (soll < sollwert_min) {
        sollwert = sollwert_min;
    }
    else {
        sollwert = soll;
    }
    //Log("neuer sollwert: %lf , geg.: %lf \n", sollwert, soll);
}

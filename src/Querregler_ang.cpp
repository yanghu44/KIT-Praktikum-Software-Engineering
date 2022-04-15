#include "Querregler_ang.h"

Querregler_ang::Querregler_ang() : PID_Regler(1.2, 0.1, 0.7, M_PI, -M_PI, 0, 5, 0.001) {	//Parameter fuer den PID-Regler setzen

}

double Querregler_ang::Querregelung_ang() {	//Optimierung des Reglers, damit das Auto stabiler fahren kann
	double dev_ang = RoadSensor[0].Route.Deviation.Ang;	//deviation angle wird von Roadsensor 0 gelesen
	retVals querOutput = regel_algo(dev_ang);	//deviation angle als Istwert in die PID regeler Algorithmus gegeben
	return querOutput.sum_out;	//Die Summe von dem PID-Anteil als Rueckgabewert 
}

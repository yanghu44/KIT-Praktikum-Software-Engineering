#pragma once

#ifndef _Abstandsregler_H__
#define _Abstandsregler_H__

#include <Vehicle/Sensor_Radar.h>
#include <CarMaker.h>
#include "PID_Regler.h"
#include <Vehicle/Sensor_ObjectByLane.h>


class Abstandsregler : public PID_Regler {
public:
	/**
	* @brief Diese Methode regelt den Abstand des Fahrzeugs zu einem vorausfahrendem Fahrzeug \n
	* 
	* Mithilfe der Geschwindigkeit des vorrausfahrenden Fahrzeugs wird der Soll-Abstand berechnet.
	* Der aktuelle Abstand wird ausgelesen und dem Regelalgorithmus uebergeben.
	* Der Output des Regelalgorithmus ist eine Soll-Geschwindigkeit in [m/s].
	* 
	*
	* @return	retVals: Die P, I, und D Anteile des Reglers werden fuer Debugging/Reglereinstellung zurueckgegeben, die Summe wird fuer den Geschwindigkeitsregler benötigt \n
	*/
	retVals abstandsRegelung(double dist, double Obj_v, double safety_dist_factor);

	/**
	* @brief Parameter fuer den PID-Regler setzen, wie z.B. P- & I- & D-Anteil \n
	* 
	* @param[in] cycle_time: Die Zykluszeit fuer den PID-Regler. 
	* 
	*/

	Abstandsregler(double cycle_time);

	/**
	* @brief Berechnung des Regelalgorithmus \n
	* 
	* @param[in] istwert: Der Ist-Abstand.
	* @return	retVals: Die P, I, und D Anteile des Reglers werden fuer Debugging/Reglereinstellung zurueckgegeben, die Summe wird fuer den Geschwindigkeitsregler benötigt \n
	*/

	retVals regel_algo(double istwert);

private:
	std::queue<double> d_queue_past;
	std::queue<double> d_queue_curr;
	double d_sum_past = 0;
	double d_sum_curr = 0;

};
#endif
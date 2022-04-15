#pragma once
#ifndef _Querregler_ang_H__
#define _Querregler_ang_H__

#include <Vehicle/Sensor_Radar.h>
#include <CarMaker.h>
#include <math.h>
#include "PID_Regler.h"


class Querregler_ang : public PID_Regler {
public:
	/**
	* @brief Optimierung des Reglers, damit das Auto stabiler fahren kann
	* 
	* @reutrn Die Summe von dem PID-Anteile von dem Regelalgorithmus
	* 
	* Deviation Angle wird von dem Sensor gelesen und als Istwert zu dem Regelalgorithmus gegeben.
	* Die Sumout Wert von dem Regelalgorithmus wird als Rueckgabewert definiert.
	*/
	double Querregelung_ang();

	/**
	* @brief Parameter fuer den PID-Regler setzen, wie z.B. P- & I- & D-Anteil. 
	* 
	* Die Sollwert wird auf 0, das Beobachtungsfenster auf 5s und das Cycle Time auf 0.001s gesetzt.
	* Das maximale und minimale Output wird jeweils auf pi und minus pi gesetzt. Das bedeutet, den Lenkrad maximal um 180 Grad gedreht werden kann.
	*/
	Querregler_ang();
};

#endif

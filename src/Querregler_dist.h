#pragma once

#ifndef _Querregler_dist_H__
#define _Querregler_dist_H__

#include <Vehicle/Sensor_Radar.h>
#include <CarMaker.h>
#include <math.h>
#include "PID_Regler.h"



struct paraQuer {
	double winkel;	///<Lenkradwinkel
	double geschwindigkeit;	///<Geschwindigkeit
};

class Querregler_dist : public PID_Regler {
public:

	/**
	* @brief Hauptregler damit das Auto in der mitte der Fahrspur fahren kann. \n
	* @param[in] limit: Die Geschwindigkeitsbegrenzung ist Eingebewert.
	* 
	* Deviation Distance wird von dem Sensor gelesen und als Istwert zu dem Regelalgorithmus gegeben. 
	* Falls eine Kurve mit bestimmte Kruemmung erscheint, wird die Laengsgeschwindigkeit soweit angepasst, dass eine angemessene Querbeschleunigung nicht ueberschritten wird.
	* 
	* @return	winkel: Die Summe von dem PID-Anteile von dem Regelalgorithmus \n
	*			geschwindigkeit: Die begrenzte Geschwindigkeit wird zurueckgegeben \n
	*/
	paraQuer Querregelung_dist(double limit);

	/**
	* @brief Parameter fuer den PID-Regler setzen, wie z.B. P- & I- & D-Anteil
	* @param[in] lanemid: Das Position von dem Anfangsspur ist Eingabewert.
	* 
	* Sollwert wird auf 0, Beobachtungsfenster auf 5s und Cycle Time auf 0.001s gesetzt.
	* Das maximale und minimale Output wird jeweils auf 5 und -5 gesetzt. Das bedeutet den Lenkrad maximal um 180 Grad gedreht werden kann.
	*/
	Querregler_dist(double lanemid);
	

};

#endif

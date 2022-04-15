#pragma once

#ifndef _Geschwindigkeitsregler_H__
#define _Geschwindigkeitsregler_H__

#include <CarMaker.h>
#include "PID_Regler.h"



class Geschwindigkeitsregler : public PID_Regler {
public:
	/**
	* @brief Diese Methode regelt die Geschwindigkeit des Fahrzeugs auf einen festgelegten Sollwert. \n
	*
	* Die Ist-Geschwindigkeit wird dem Regelalgorithmus uebergeben, der Output des Regelalgortihmus wird durch die Variable "next_Gas" in einen Wert fuer Gas oder Bremse des Fahrzeugs uebersetzt.
	* 
	* @return	retVals: Die P, I, und D Anteile des Reglers werden fuer Debugging/Reglereinstellung zurueckgegeben \n			
	*/
	retVals geschwindigkeitsRegelung();

	/**
	* @brief Parameter fuer den PID-Regler setzen, wie z.B. P- & I- & D-Anteil \n
	* 
	* @param[in] limit: Die Geschwindigkeitsbegrenzung (durch Verkehrsschilder oder ein situationsbedingter Wert).
	* @param[in] cycle_time: Die Zykluszeit fuer den Geschwindigkeitsregler, wird verwendet zur Berechnung von nextGas.			
	* nextGas wird auf 0 gesetzt.
	* 
	*/
	Geschwindigkeitsregler(double limit, double cycle_time);
	/**
	* @brief Setzt den Sollwert fuer die Geschwindigkeit in [m/s] \n
	* 
	* @param[in] soll: Die Geschwindigkeitsbegrenzung (durch Verkehrsschilder oder ein situationsbedingter Wert).
	* Sollwerte die groesser oder kleiner als der maximale/minimale Sollwert sind, werden auf das
	* entsprechende Maximum/Minimum begrenzt.
	*
	*/

	void set_sollwert(double soll);

private:
	double nextGas;


};
#endif

//changed

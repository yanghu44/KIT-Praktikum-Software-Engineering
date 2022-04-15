#pragma once

#ifndef _UMGEBUNGSUEBERWACHUNG_H__
#define _UMGEBUNGSUEBERWACHUNG_H__

#include <Carmaker.h>
# include <Vehicle/Sensor_Radar.h>
#include <Vehicle/Sensor_TSign.h>
#include <Vehicle/Sensor_Road.h>
#include <iostream>
#include <deque>
#include <vector>
#include <Vehicle/Sensor_ObjectByLane.h>

#include "Geschwindigkeitsregler.h"
#include "Abstandsregler.h"
#include "Querregler_dist.h"
#include "Querregler_ang.h"
#include "PID_Regler.h"

//changed

struct sensor_data {
	double limit = 36.0;
	int aktiver_regler = 0;
	double turn_max_speed = 36.0;
	retVals regler_values;
};


class Umgebungsueberwachung {

public:
	
	/**
		* @brief Dieser Methode entscheidet, welche Fahrmodus ausgefuehrt werden soll.
		* @param sim_time: vergangene Zeit in der Simulation in ms
		* @param soll_wechsel: manuelle Kontrollwert fuer den Wechsel des Fahrspur
		* @param driver_limit: manuelles Geschwindigkeitslimit
		* @param driver_safety_dist: Faktor zur Einstellung des Sicherheitsabstands bei Folgefahrt und Ueberholvorgaengen
		* 
		* Falls ein manuelles Driver limit festgelegt wurde, wird dieses eingelesen und als Sollgeschwindigkeit uebernommen. \n
		* Wenn sich kein Auto in der Spur vor dem Ego-Fahrzeug befindet oder das Fahrzeug schneller als das berechnete/vorgegebene Limit faehrt, ist der Geschwindigkeitsregler aktiv und regelt auf das Limit. \n
		* 
		* Wenn sich ein Auto in der Spur dem Egofahrzeug befindet, und es schnell genug faehrt um nicht ueberholt werden zu muessen, ist der Abstandsregler aktiv 
		* und regelt den Abstand auf einen berechneten Sicherheitsabstand. \n
		* 
		* 
		* Beim Spurwechsel wird zwei Modi mauelle und automatische Modi definiert.\n
		* Manuell: Der manuell eingegebene Wechselbefehl wird gelesen.\n
		* Automatisch: verschiedene Szenarien werden definiert, bei der das Auto automatisch ueberholen kann:\n
		* 1. Es gibt Objekt vor dem Egofahrzeug und faehrt sehr langsam:\n
		* Dieses Objekt wird als Hindernis erkannt. Mittels Sensoren wird es gecheckt, ob es noch genug Platz auf linke/rechte Spur gibt.
		* Wenn es genug Platz gibt, wird ein entsprende Wechselbefehl mittels Algorithmus automatisch erstellt. \n
		* 2. Es gibt Objekt vor dem Egofahrzeug und faehrt nur ein bisschen langsamer.\n
		* Das Objekt wird als Auto erkannt. In dem Fall wird die linke Spur gecheckt. Falls es genug Platz gibt, wird ein Wechselbefehl uebergeben. 
		* Diese Wechselbefehl wird von der Methode "spur_wechsel" aufgerufen. \n
		* 3. Das Egofahrzeug wird automatisch auf die Startspur zurueckgewechselt\n
		* Es wird ueberprueft, ob das Auto gerade auf der Startspur faehrt. Falls es nicht der Fall ist, wird ein Wechselbefehl nach der Betrachtung des Verkehrssituations automatisch erstellt.
	*/
	void Moduswahl(double sim_time, int soll_wechsel, double driver_limit, double driver_safety_dist);

	/**
	* @brief Die Methode liest den Wechselbefehl an, und fuehrt die wchselaktion durch
	* @param[in] wechsel_LR: Der Befehl von Spurwechsel. links = 1, rechts = 2, stil bleiben = 0.
	*
	* Erstmal unterscheidet die Methode das Auto zu welcher Spur zu wechseln.
	* Falls die Spur befahrbar waere, dann werden der wechselbefehl durch change_lane_counter zum naechsten Schritt uebermittelt.
	* Die Differenz zwischen change_lane_counter und prev_change_lane_counter zeigt die Wechselrichtung an, danach wird der Befehl durchgefuehrt.
	* Der Sollwert der "Deviation Distance" wird im Anhang der Spurbreite bestimmt, dann approximiert der change_lane_parameter langsam zu der Solldistanz (in 6 Sekunden).
	* Nachdem der Prozess endet, wird change_lane wieder auf false gesetzt, bedeutet der Wechselprozess zu Ende ist.
	*
	*/
	void spur_wechsel(int wechsel_LR);

	/**
	* @brief Der Wert von "Wechsel" wird gelesen
	* @return Der Wert von Wechsel wird zurueckgegeben.
	*
	*/
	int get_wechsel();

	/**
	* @brief In diser Methode werden die durch Moduswahl festgelegten Aktionen ausgefuehrt.
	* @param[in] sim_time: Die aktuelle Simulationszeit in ms.
	* 
	* Abstandsregler aktiv: \n
	* Der Abstandsregler wird nur in den durch cycle_time bestimmten Zeitabstaenden aufgerufen, in der Zwischenzeit werden Gas und Bremse konstant gehalten. \n
	* Die vom Abstandsregler vorgegebene Geschwindigkeit wird an den Geschwindigkeitsregler als Sollwert uebergeben, der daraufhin aufgerufen wird. \n \n
	* 
	* Geschwindigkeitsregler aktiv: \n
	* Geschwindigkeit wird auf festgelegtes/berechnetes Limit geregelt.
	* 
	* Querregler (immer aktiv): \n
	* 
	* Wenn ein Wechsel durchgefuehrt werden soll, werden die entsprechenden Parameter an den Spurhalte- und Distanz-Querregler uebergeben und diese aufgerufen.
	* Ansonsten wird das Ego-Fahrzeug nur in der Mitte der aktuellen Spur gehalten.
	*
	*/
	void regler_aufruf(double sim_time);

	/**
	* @brief Ausgewaehlte Daten werden ausgelesen um sie in Carmaker anzeigen lassen zu koennen.
	* @return Die ausgewaehlten Daten.
	*
	*/

	sensor_data get_sensor_data();


	/**
	*@brief Initialisierung der Parameters
	*/
	void init();
private:
	double limit = 36;	///<Geschwindigkeitsbeschraenkung

	/**
	* @brief Methode zur Erkennung von Tempo-limit Schildern \n
	* 
	* Falls ein Tempo-Limit erkannt wird, wird der angezeigte Wert als neue Geschwindigkeitsbegrenzung festgelegt. \n
	* Es wird immer nur das naechste Schild beachtet.
	* @return Indikator ob Schilder erkannt wurden oder nicht.
	*
	*/

	int recognize_speed_sign();

	// fuer Spur_wechsel
	int prev_change_lane_counter = 0;	///< Der Wert von change_lane_counter aus dem letzten Takt.
	int change_lane_counter = 0;	///< Der zeigt an, wie viele Male das Auto bzgl. der originale Spur Spur gewechselt hat. links = negativ, rechts = positiv.
	int wechsel;	///<Spur wechsel 1:nach links & 2:nach rechts & 0:still bleiben
	double change_lane_parameter = 0.0;	///< Das Offset der Dev-Dist zwischen den originale Dev-Dist und soll Dev-Dist.
	double lane_width[2] = { 0.0 };	///<Spur Breite 0:linke Spur, 1:rechte Spur
	sensor_data sens_data;
	bool change_lane = false;	///<initialisierung von Spurwechsel Befehl. Es zeigt an, ob eine Laufende Wechselaktion existiert.
};







#endif

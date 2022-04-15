#pragma once

#ifndef _PID_Regler_H__
#define _PID_Regler_H__
#include<cmath>
#include<iostream>
#include<queue>
#include<array>

struct retVals {
	double sum_out;
	double p_out;
	double i_out;
	double d_out;
};

	class PID_Regler {

	protected:
		double K_p;			///< Der P-Parameter des PID-Reglers
		double K_i;			///< Der I-Parameter des PID-Reglers
		double K_d;			///< Der D-Parameter des PID-Reglers

		double sollwert;		///< Der gewuenschte Sollwert
		double sollwert_min;	///< Der niedrigste moegliche Sollwert
		double sollwert_max;	///< Der groesste moegliche Sollwert
		double max;				///< Die maximale Stellgroesse
		double min;				///< Die minimale Stellgroesse

		double cycle_time;		///< Der Zeitschritt zwischen den Berechnungen (Zykluszeit)


		double i_sum = 0.0;		///< Das Integral fuer die Berechnung des I-Anteils
		std::queue<double> i_queue;  ///< Die I-Anteile zur Berechnung des Integrals
		double e_prev = 0.0;	///< Die Abweichung im letzten Schritt zur Berechnung des D-Anteils
		int i_window_length;	///< Die Anzahl der Zeitschritte zur Berechnung des Integrals

	public:
		/**
		* @brief Parameter fuer den PID-Regler setzen \n
		*
		* @param[in] K_p: 
		* @param[in] K_i:
		* @param[in] K_d: 
		* @param[in] max: 
		* @param[in] min:
		* @param[in] i_window_length:
		* @param[in] cycle_time: 
		*/

		PID_Regler(double K_p, double K_i, double K_d, double max, double min, double soll, int i_window_length, double cycle_time);

		/**
		* @brief Parameter fuer den PID-Regler setzen  (Konstruktor fuer Geschwindigkeits & Abstandsregler\n
		*
		* @param[in] K_p:
		* @param[in] K_i:
		* @param[in] K_d:
		* @param[in] max:
		* @param[in] min:
		* @param[in] i_window_length:
		* @param[in] cycle_time:
		* @param[in] sollwert_min:
		* @param[in] sollwert_max:
		*/

		PID_Regler(double K_p, double K_i, double K_d, double max, double min, double soll, int i_window_length, double cycle_time, double sollwert_min, double sollwert_max);

		/**
		* @brief  Methode zum setzen des aktuellen Sollwerts \n
		* @param[in] soll: Der Sollwert
		*/

		void set_sollwert(double soll);

		/**
		* @brief  Methode zum setzen des maximalen Sollwerts \n
		* @param[in] soll: Der maximalen Sollwert
		*/
		void set_sollwert_max(double soll);

		/**
		* @brief  Methode zur Berechnung der Stellgroesse \n
		* @param[in] istwert: Der Ist-Wert der zu regelnden Groesse
		*/

		retVals regel_algo(double istwert);



	};

	//changed


#endif

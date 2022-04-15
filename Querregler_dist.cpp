#include "Querregler_dist.h"

Querregler_dist::Querregler_dist(double lanemid) : PID_Regler(1.5, 0.5, 0.7, 5.0, -5.0, lanemid, 5, 0.001) { //1.5  0.3   0.7
	
}


paraQuer Querregler_dist::Querregelung_dist(double limit) { //Hauptregler damit das Auto in der mitte der Fahrspur fahren kann
	double dev_dist = RoadSensor[0].Route.Deviation.Dist;   // read the deviation distance
    paraQuer querOutput;    // struct paraQuer: double winkel, double geschwindigkeit 
	retVals regler_values = regel_algo(dev_dist);   //deviation distance als Istwert in die PID regeler Algorithmus gegeben
    querOutput.winkel = regler_values.sum_out;  //Die Summe von dem PID-Anteil zu dem Parameter winkel gegeben

    //Querreglung mit Geschwindigkeitskontroll
    if (abs(RoadSensor[1].Route.CurveXY) >= 0.001) {    //falls eine Kurve mindestens eine Kruemmung(curvature) von 0.001 gibt (Radius <= 1000m)
        //hoehste Kurve Geschwindigkeit v = sqrt( u * g / K)  u: Reibungskonstante    g: Erdbeschleunigung    K: Kruemmung(curvature) 
        //0.8 * maximale Wert, damit eine Toleranz gibt
        if (0.8 * sqrt(1.0 * 9.8 / abs(RoadSensor[1].Route.CurveXY) <= limit)) {    //falls die hoehst erlaubte Geschwindigkeit von der Kurve kleiner als Geschwindigkeitsbeschraenkung ist
            querOutput.geschwindigkeit = 0.8 * sqrt(1.0 * 9.8 / abs(RoadSensor[1].Route.CurveXY));  //wird die hoehst erlaubte Geschwindigkeit von der Kurve gesetzt
        }
        else {  //die Geschwindigkeit wird trotzdem verringert, damit ein besseres Resultat vom Regler erscheint
            querOutput.geschwindigkeit = 0.8 * limit;
        }
    }
    else {  //falls sehr kleine Kruemmung gibt (fast geradeaus fahren), wird die Geschwindigkeit zu der Beschraenkung angepasst
        querOutput.geschwindigkeit = limit;
    }

	return querOutput;  //winkel und geschwindigkeit als Rueckgabewert
}


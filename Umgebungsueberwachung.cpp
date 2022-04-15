#include "Umgebungsueberwachung.h"

double cycle_time = 0.06;
int aktiver_regler = 0;
double curr_gas;
double curr_brake;
double limit = 36;
double limit_mem = 36;
double safety_dist_factor = 6;


/*Für Querregler*/
paraQuer distOutput;
double angOutput;
double lanemid = 0;

Querregler_dist qd_regler(lanemid);
Querregler_ang qa_regler;
//Als Eingabe für Methode spur_wechsel
//int change_lane_counter = 0;
//int prev_change_lane_counter = 0;
//int wechsel = 0;
//double change_lane_parameter = 0;
//int counter_tolerance[6] = { 0 };

int current_closest_sign_id = 0;
Abstandsregler dist_regler(cycle_time);
Geschwindigkeitsregler v_regler(limit, 0.001);
Geschwindigkeitsregler va_regler(limit, cycle_time);

void Umgebungsueberwachung::init() {
    cycle_time = 0.06;
    aktiver_regler = 0;
    curr_gas = 0.0;
    curr_brake = 0.0;
    limit = 36;
    lanemid = 0.0;
    change_lane_counter = 0;
    prev_change_lane_counter = 0;
    change_lane_parameter = 0.0;
    lane_width[2] = { 0.0 };
    wechsel = 0;
    limit_mem = 36;
    safety_dist_factor = 6; 
    change_lane = false;
    current_closest_sign_id = 0;
}



int Umgebungsueberwachung::recognize_speed_sign() {
    for (int i = 0; i < TSignSensor[0].nSign; i++) {

        if (TSignSensor[0].Sign[i].ds_p <= TSignSensor[0].Sign[current_closest_sign_id].ds_p) {
            current_closest_sign_id = i;
            if (int(TSignSensor[0].Sign[current_closest_sign_id].main.val[1]) == -1) {          //wenn Geschwindigkeitsbegrenzung aufgehoben wird
                limit = 36.0;                                                                   //Limit = Richtgeschwindigkeit ( 130kmh)
            }
            else if ((TSignSensor[0].Sign[current_closest_sign_id].main.val[0]) != 0){          // Wenn das erkannte Schild ein Tempo-limit ist
                limit = double(TSignSensor[0].Sign[current_closest_sign_id].main.val[0]) / 3.6; //neues Tempolimit wird eingelesen (in [m/s])
            }
            
            limit_mem = limit;                          
            //Log("new limit: %d \n", int(limit * 3.6));
            sens_data.limit = limit;
            v_regler.set_sollwert(limit);
            va_regler.set_sollwert_max(limit);
        }

    }
    return(1);
}


void Umgebungsueberwachung::spur_wechsel(int wechsel_LR) {
    wechsel = wechsel_LR;
   
    //wechsel = 1, nach links abbiegen, OBL Sensor[Nummer 0].Lane[linke Spur 0][erste Spur 0]
    if (wechsel == 1) {
        if(ObjByLane[0].Lane[0][0].Type == 0) {     //ueberpruefen ob linke Spur fahrbar ist, fahrbar = 0
            prev_change_lane_counter = change_lane_counter; //die vorherige gewechselte Anzahl von Spuren 
            change_lane_counter -= 1;   //ein mal nach links abbiegen, die aktuelle gewechslte Anzahl von Spuren aktualisieren
            lane_width[0] = ObjByLane[0].Lane[0][0].Width;  //linke Spurbreite aus Sensor lesen
            wechsel = 0;    //Parameter wechsel zuruecksetzen, damit das Auto nicht immer eine wechsel Befehl bekommen wird
        }
    }

    //wechsel = 2, nach rechts abbiegen, OBL Sensor[Nummer 0].Lane[rechte Spur 2][erste Spur 0]
    if (wechsel ==2) {
        if (ObjByLane[0].Lane[2][0].Type == 0) {    //ueberpruefen ob rechte Spur fahrbar ist, fahrbar = 0
            prev_change_lane_counter = change_lane_counter;
            change_lane_counter += 1;   //ein mal nach rechts abbiegen, die aktuelle gewechslte Anzahl von Spuren aktualisieren
            lane_width[1] = ObjByLane[0].Lane[2][0].Width;  //rechte Spurbreite aus Sensor lesen
            wechsel = 0;
        }
    }

    if (change_lane_counter < prev_change_lane_counter) {   //nach links abbiegen, falls aktuelle counter kleiner als vorherige counter
        if (lane_width[0] * change_lane_counter < change_lane_parameter) {  //offset = linke Spur Breite * Anzahl der gewechslten Spuren
            change_lane_parameter -= lane_width[0] / 6000;  //lane breite durch 6000ms, um in 6s Spurwechsel realisieren zu koennen
        }
        else {  //nach dem erfolgreiche Wechsel, Spurwechsel Befehl zureck auf false setzen
            change_lane = false;
            prev_change_lane_counter = change_lane_counter;
            Log("Spurwechsel fertig \n");
        }
    }
    else if (change_lane_counter > prev_change_lane_counter) {  //nach rechts abbiegen, falls aktuelle counter groesser als vorherige counter
        if (lane_width[1] * change_lane_counter > change_lane_parameter) {  //offset = rechte Spur Breite * Anzahl der gewechslten Spuren 
            change_lane_parameter += lane_width[1] / 6000;
        }
        else {
            change_lane = false;
            prev_change_lane_counter = change_lane_counter;
            Log("Spurwechsel fertig \n");
        }
    }

}

int Umgebungsueberwachung::get_wechsel() {  //um das Parameter wechsel zu aktualisieren
    return wechsel;
}

void Umgebungsueberwachung::Moduswahl(double sim_time, int soll_wechsel, double driver_limit, double driver_safety_dist) {

	recognize_speed_sign();

    if (driver_limit != -1) {    //vom Fahrer vorgegebene Richtgeschwindigkeit
        limit = driver_limit;
        v_regler.set_sollwert(limit);
        va_regler.set_sollwert_max(limit);
    }
    else {
        limit = limit_mem;
    }


    if (driver_safety_dist != -1) {                //vom Fahrer vorgegebner Sicherheitsabstand         
        safety_dist_factor = driver_safety_dist;
    }


	if (ObjByLane[0].Lane[1][0].nObjF == 0) { //falls kein Auto in der Spur vor dem Egofahrzeug erkannt wird
        aktiver_regler = 0;                   // Geschwindigkeitsregler regelt auf berechnetes/eingestelltes Limit
	}
	else {
		double front_obj_v = ObjByLane[0].Lane[1][0].ObjFront[0].VelLong;   //die Geschwindigkeit des vorausfahrenden Fahrzeugs
		if (front_obj_v > limit) {              //falls das vorausfahrende Fahrzeug schneller als das Limit fährt
			aktiver_regler = 0;                 //Geschwindigkeitsregler Regler aktiv
		}
		else {                                   //falls voraufahrendes Fahrzeug langsamer als das Limit fährt
			aktiver_regler = 1;                  // Folgefahrt -> Abstandsregler aktiv
		}
    sens_data.aktiver_regler = aktiver_regler;
	}
    // Hindernis Erkennung & Spurwechsel
    if (soll_wechsel != 0) {    //pruefen, ob das Parameter wechsel in DVA_VC manuell eingegeben wird
        wechsel = soll_wechsel;
        change_lane = true;
    }
    else {  //automatisierte fahren

        if (Vehicle.v * safety_dist_factor > ObjByLane[0].Lane[1][0].ObjFront[0].sMin && Vehicle.v - ObjByLane[0].Lane[1][0].ObjFront[0].VelLong > 40 / 3.6 && ObjByLane[0].Lane[1][0].nObjF != 0 && change_lane == false) { //Hindernis vor dem Auto erkannt. die V_Differenz mit dem vorrausfahrendem Auto >= 40 km/h und es gibt keine zurzeitige Wechsel Aktion
            //Log("Hindernis erkannt. \n");
            //links ueberholen hat vorran. zuerst ueberpruefen ob links abbiegen moeglich, vorner && hinter, Sicherheitsabstand mit 8 Sekunden * aktuetuelle Ego-Fahrzeugs Geschwindigkeit erfuellt || keine Autos erkannt auf der linker Spur
            if ((Vehicle.v * safety_dist_factor < ObjByLane[0].Lane[0][0].ObjFront[0].sMin || ObjByLane[0].Lane[0][0].nObjF == 0) && (Vehicle.v * safety_dist_factor < abs(ObjByLane[0].Lane[0][0].ObjRear[0].sMin) || ObjByLane[0].Lane[0][0].nObjR == 0)) {
                wechsel = 1;
                change_lane = true;
                Log("Ausweichen nach links einleiten. \n");

            }
            //danach auf den rechte Spur ueberpruefen, vorner && hinter, Sicherheitsabstand mit 8 Sekunden * aktuetuelle Ego-Fahrzeugs Geschwindigkeit erfuellt || keine Autos erkannt auf der rechte Spur
            else if ((Vehicle.v * safety_dist_factor < ObjByLane[0].Lane[2][0].ObjFront[0].sMin || ObjByLane[0].Lane[2][0].nObjF == 0) && (Vehicle.v * safety_dist_factor < abs(ObjByLane[0].Lane[2][0].ObjRear[0].sMin) || ObjByLane[0].Lane[2][0].nObjR == 0)) {
                wechsel = 2;
                change_lane = true;
                Log("Ausweichen nach rechts einleiten. \n");

            }
        }
        else if (Vehicle.v * safety_dist_factor > ObjByLane[0].Lane[1][0].ObjFront[0].sMin && ObjByLane[0].Lane[1][0].ObjFront[0].VelLong < limit - 10 / 3.6 && ObjByLane[0].Lane[1][0].nObjF != 0 && change_lane == false && ObjByLane[0].Lane[0][0].Type == 0) {        //lansgam fahrendes Auto erkannt 10kmh langsamer -> links überholen (wenn möglich)
            //Log("Langsames Auto erkannt \n");
            //links ueberholen
            if ((Vehicle.v * safety_dist_factor < ObjByLane[0].Lane[0][0].ObjFront[0].sMin || ObjByLane[0].Lane[0][0].nObjF == 0) && (Vehicle.v * safety_dist_factor < abs(ObjByLane[0].Lane[0][0].ObjRear[0].sMin) || ObjByLane[0].Lane[0][0].nObjR == 0)) {
                wechsel = 1;
                change_lane = true;
                Log("Links ueberholen einleiten. \n");
            }
        }
        else if (change_lane == false && change_lane_counter != 0) {          //kein Hindernis/langsames Auto erkannt -> zurück in Ausgangsspur wechseln (wenn möglich)
            //Log("Kein Objekt erkannt");
            //Log("front dist: %lf , rear dist: %lf \n", ObjByLane[0].Lane[2][0].ObjFront[0].sMin, abs(ObjByLane[0].Lane[2][0].ObjRear[0].sMin));
            if ((Vehicle.v * safety_dist_factor < ObjByLane[0].Lane[0][0].ObjFront[0].sMin || ObjByLane[0].Lane[0][0].nObjF == 0) && (Vehicle.v * safety_dist_factor < abs(ObjByLane[0].Lane[0][0].ObjRear[0].sMin) || ObjByLane[0].Lane[0][0].nObjR == 0) && change_lane_counter > 0 ) {
                wechsel = 1;    //nach links
                change_lane = true;
                Log("Spurwechsel nach links einleiten um in Ausgangsspur zu gelangen. \n");
            }
            else if ((Vehicle.v * safety_dist_factor < ObjByLane[0].Lane[2][0].ObjFront[0].sMin || ObjByLane[0].Lane[2][0].nObjF == 0) && (Vehicle.v * safety_dist_factor < abs(ObjByLane[0].Lane[2][0].ObjRear[0].sMin) || ObjByLane[0].Lane[2][0].nObjR == 0) && change_lane_counter < 0 ) {
                wechsel = 2;    //nach rechts
                change_lane = true;
                Log("Spurwechsel nach rechts einleiten um in Ausgangsspur zu gelangen.");
            }
        }
    }
    regler_aufruf(sim_time);
	//bedingungen für Spurwechsel überprüfen, regler aufrufen
}




void Umgebungsueberwachung::regler_aufruf(double sim_time) {

    //ABSTANDSREGLER
    if (aktiver_regler == 1) {
        if (int(sim_time * 1000) % int(cycle_time * 1000) == 0) {       //Only execute in timesteps according to cycle time

            double dist = ObjByLane[0].Lane[1][0].ObjFront[0].sMin;     //Abstand zum vorausfahrendem Fahrzeug
            double Obj_v = ObjByLane[0].Lane[1][0].ObjFront[0].VelLong; //Geschwindigkeit des vorausfahrenden Fahrzeugs
            retVals dist_regler_output = dist_regler.abstandsRegelung(dist, Obj_v, safety_dist_factor);  //Aufruf des Abstandsregelalgorithmus



            double soll_v_diff = dist_regler_output.sum_out;

            //Log("Measured speed of car in front: %lf \n", Obj_v);
            double soll_v = Obj_v - soll_v_diff;                       //Berechnung der Sollgeschwindigkeit mit der Stellgröße des Abstandsreglers
            va_regler.set_sollwert(soll_v);                             
            //Log("Vom Abstandsregler bestimmer Gerschwindigkeits-Sollwert: %lf \n", soll_v);
            sens_data.regler_values = va_regler.geschwindigkeitsRegelung();                       //Aufruf des Geschwindigkeitsreglers

            curr_gas = VehicleControl.Gas;                              //Speichern des aktuellen Gaswerts
            curr_brake = VehicleControl.Brake;                          //Speichern des aktuellen Bremswerts
            //Log("curr_gas: %lf, curr_brake, %lf: \n", curr_gas, curr_brake);
        }
        else {                                                          //Zwischen den von cycle_time bestimmten Zeitschritten
            VehicleControl.Gas = curr_gas;                              //Gas konstant halten
            VehicleControl.Brake = curr_brake;                          //Bremse konstant halten
            //Log("curr_gas: %lf, curr_brake, %lf: \n", curr_gas, curr_brake);
        }
    }


    //GESCHWINDIGKEITSREGLER
    if (aktiver_regler == 0) {
        sens_data.regler_values = v_regler.geschwindigkeitsRegelung();
    }



    //QUERREGLER
    lanemid = RoadSensor[0].Act.tMidLane;   //Start Positon bezueglich auf Reference linie
    distOutput = qd_regler.Querregelung_dist(limit);    //Querregler_dist output
    angOutput = qa_regler.Querregelung_ang();           //Querregler_ang output
    double reglerOutput = distOutput.winkel * 0.7 + angOutput * 0.3;    //das Output wird abgewaegt und eine neue Wert von dem gesammtem Regeleroutput geben
    VehicleControl.Steering.Ang = reglerOutput; //die Wert von dem abgewaegtem Regleroutput zu Lenkrad Winkel uebergeben
    spur_wechsel(wechsel);      //das in User.cpp manuell gestellte Wert von wechsel wird zu dem Funktion spur_wechsel uebergeben. ansonsten ist wechsel gleich
    qd_regler.set_sollwert(lanemid - change_lane_parameter);    //Sollwert = Start Position - Offset 
    wechsel = get_wechsel();    //Parameter wechsel wird aktualisiert

    sens_data.turn_max_speed = distOutput.geschwindigkeit;
    v_regler.set_sollwert_max(distOutput.geschwindigkeit);    //maximale Geschwindigkeit für v_regler festlegen
    va_regler.set_sollwert_max(distOutput.geschwindigkeit);   //maximale Geschwindigkeit für va_regler festlegen
    v_regler.set_sollwert(distOutput.geschwindigkeit);        //Während Freifahrt wird hier der Sollwert für die Geschwindigkeit gesetzt
   
    
}

sensor_data Umgebungsueberwachung::get_sensor_data() {
    return sens_data;
}
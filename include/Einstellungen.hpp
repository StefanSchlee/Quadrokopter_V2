#ifndef EINSTELLUNGEN_HPP
#define EINSTELLUNGEN_HPP

#define BLUETOOTH   //Bluetooth Ausgabe möglich: Mit drehregler wird gestartet, Output siehe loop, Rx2=16
#define SPANNUNGSMESSUNG //wird Spannung überprüft
//#define GPIO_LOOP_TIMING //wird Loopdauer an GPIO ausgegeben
//#define NO_ESC_OUTPUT   //immer 0 throttle ausgeben

#define Max_Input_angle 20.0f //Maximaler Ausschlag  //20
#define Max_Input_yaw 120.0f  //Maximale Drehgeschwindigkeit in grad/s
#define Max_Throttle 700      //Maximale Gasstellung 0 .. 1000

#define USB_BAUD 500000

/*********  Zeit  **********/

#define ABTASTZEIT_US 1000                                   //Abstastzeit für Alles in µs
static const float Abtastzeit_s = ABTASTZEIT_US / 1000000.0; //Abtastzeit umgerechnet in Sekunden


/*********  Bluetooth  **********/
#define BLUETOOTH_INTERVALL 10  //Sende Bluetooth alle INTERVALL Male

#endif // EINSTELLUNGEN_HPP

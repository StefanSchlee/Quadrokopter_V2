#ifndef REGLER_HPP
#define REGLER_HPP

#include <Arduino.h>
#include <DigitalFilter.h>

/**
 * Winkelgeschwindigkeit Regler
 * P: Proportionale Verstärkung Regeldifferenz
 * I: Integrale Verstärkung Regeldifferenz
 * Pvorst:Verstärkung der Vorsteuerung der Ableitung der Sollrate
 */
class AngVelCtrl
{
private:
    float P;    //Reglerverstärkung proportional
    float I;    //Reglerverstärkung integral
    float D;    //Reglerverstärkung differenzial
    float Pvorst;   //Reglerverstärkung Vorsteuerung
    Biquad gyro_filter;  //Tiefpassfilter für ist Drehrate
    Biquad D_filter;    //zusätzlicher Tiefpass für D Anteil
    PT1 vorst_filter;   //filter für Vorsteuerung
    float I_Anteil; //Integralspeicher 
    float lastSollRate; //letzte Sollrate für Ableitung für Vorsteuerung
    float lastError;    //letzte Regeldifferenz für D-Anteil

public:
    AngVelCtrl(float P, float I, float D, float Pvorst);
    float update(float sollRate, float istRate);
    void resetIntegral();
    float getDebugValue();
    float get_gyro_filter();
}; 

/**
 * Winkel Regler 
 * Stellgröße: Winkelgeschwindigkeit
 */
class AngCtrl
{
private:
    float P;    //Reglerverstärkung
public:
    AngCtrl(float P);
    float update(float sollWinkel, float istWinkel);
};







#endif // REGLER_HPP

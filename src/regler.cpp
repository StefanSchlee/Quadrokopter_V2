#include "regler.hpp"
#include "Einstellungen.hpp"


/**================================================== *
 * ==========  Winkelgeschwindigkeit Regler  ========== *
 * ================================================== */

/**
 * Initialisiert Regler mit Reglerverstärkungen
 */
AngVelCtrl::AngVelCtrl(float P, float I,float D, float Pvorst)
    : P(P),
      I(I),
      D(D),
      Pvorst(Pvorst),
      gyro_filter(0.2427, -0.1536, 0.2427, -1.106058717681099, 0.437848515141685),
      D_filter(0.2427, -0.1536, 0.2427, -1.106058717681099, 0.437848515141685),
      vorst_filter(80, 1000),
      I_Anteil(0.0f),
      lastSollRate(0.0f),
      lastError(0.0f)
{
}

/**
 * Regelalgorithmus
 * sollRate: soll Winkelgeschwindigkeit
 * istRate: gemessene Winkelgeschwindigkeit
 * 
 * return: Stellgröße
 */
float AngVelCtrl::update(float sollRate, float istRate)
{
    //Gyro Tiefpassfilter
    gyro_filter.update(istRate);

    //Regeldifferenz
    float e = sollRate - gyro_filter.get_Value(); 

    //Vorsteuerung
    float diffSollRate = (sollRate - lastSollRate) / Abtastzeit_s;
    lastSollRate = sollRate;
    float V_Anteil =  vorst_filter.update(diffSollRate) * Pvorst;

    //Proportional
    float P_Anteil = e * P;

    //Integral
    I_Anteil += e * I * Abtastzeit_s;

    //Derivative
    float diffError = (e - lastError) / Abtastzeit_s;
    lastError = e;
    float D_Anteil = D_filter.update(diffError * D);

    //Gesamt und filter
    return P_Anteil + D_Anteil + V_Anteil + I_Anteil;
}

/**
 * Integral-Anteil auf 0 setzen
 */
void AngVelCtrl::resetIntegral()
{
    I_Anteil = 0;
}

/**
 * Einen Wert zum debuggen ausgeben
 */
float AngVelCtrl::getDebugValue()
{
    return gyro_filter.get_Value();
}

/**
 * Den Gefilterten Gyro wert ausgeben
 */
float AngVelCtrl::get_gyro_filter()
{
    return gyro_filter.get_Value();
}



/**================================================== *
 * ==========  Winkel Regler  ========== *
 * ================================================== */

AngCtrl::AngCtrl(float P) : P(P)
{
}

float AngCtrl::update(float sollWinkel, float istWinkel)
{
    float e = sollWinkel - istWinkel;
    return e * P;
}
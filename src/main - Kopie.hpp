#include <Arduino.h>
#include <SPI.h>
#include <MPU9250_SPI.h>
#include <IMU_KalmanFilter.h>
#include <PPMReader_esp32.h>
//#include <ESC_FastPWM_esp32.h>
#include <ESC_DSHOT_ESP32.h>
#include <esp32-hal-ledc.h>
#include <esp32-hal-timer.h>
#include <DigitalFilter.h>
#include <SerialDebug.h>
#include "Einstellungen.hpp"
#include "regler.hpp"

/***********************************************
 * Einstellungen in Einstellungen.hpp
 * *********************************************/
//////// PID Parameter ////////
float Pang_pitch = 10;
float Ivel_Y = 0.3;  //0.2
float Pvel_Y = 0.3;
float Dvel_Y = 0.015;
float Kvorst_Y = 0.00;

float Pang_roll = 10;
float Ivel_X = 0.3; //0.2 
float Pvel_X = 0.35;
float Dvel_X = 0.015; //0.01
float Kvorst_X = 0.00;

float Pvel_Z = 1;
float Ivel_Z = 0.5;
float Dvel_Z = 0;
float Kvorst_Z = 0;

/*********************************************
 * *******************************************/

//////// Funktionsheader ////////
void get_Input();
void get_Lage_Kalman();
void PID();
int16_t get_min_throttle();
void Init_Timer();
void IRAM_ATTR loop_timerISR();
void floatWriteSerial1(float data);
float map_long_float(int32_t x, int32_t in_min, int32_t in_max, float out_min, float out_max);

#if defined(BLUETOOTH)
void floatWrite(float data);            //Bluetooth
void Bluetooth_Driver(void *parameter); //Bluetooth
#endif                                  // BLUETOOTH

#if defined(SPANNUNGSMESSUNG)
void SpannungTest(void *parameter);
void Piepen(void *parameter);
void Init_Spannungsmessung();
#endif // SPANNUNGSMESSUNG

//////// Pinbelegung ////////
const uint8_t Spannung_Pin = 25;
const uint8_t GPIO_Output_Pin = 2;
const uint8_t PPMInputPin = 18;
const uint8_t SummerPin = 32;

/**================================================== *
 * ==========  Globale Variablen  ========== *
 * ================================================== */
const int MPUaddress = 0x68; //I2C address

volatile uint8_t loop_flag = 0; //interrupt flag
hw_timer_t *loop_timer = NULL;  //timer struct

uint32_t messStart;   //debug
float testausgabe[3]; //debug
PT1 testPT1 = PT1(100, 1000);

#if defined(BLUETOOTH)
float bluetoothAusgabe[4];
bool sende = false;
bool endesende = false;
uint8_t bluetoothCounter = 0; //zähler um Daten nur alle X samples zu senden(Einstellungen)
QueueHandle_t myqueue;
#endif // BLUETOOTH

#if defined(SPANNUNGSMESSUNG)
TaskHandle_t Piepen_handle;
#endif // SPANNUNGSMESSUNG

/*********  Regler Objekte  **********/
AngVelCtrl angVelCtrlX = AngVelCtrl(Pvel_X, Ivel_X, Dvel_X, Kvorst_X);
AngVelCtrl angVelCtrlY = AngVelCtrl(Pvel_Y, Ivel_Y, Dvel_Y, Kvorst_Y);
AngVelCtrl angVelCtrlZ = AngVelCtrl(Pvel_Z, Ivel_Z, Dvel_Z, Kvorst_Z);

AngCtrl angCtrlX = AngCtrl(Pang_roll);
AngCtrl angCtrlY = AngCtrl(Pang_pitch);

//MPU
float Total_angle[2];   //Winkel 0 ist Pitch, 1 ist Roll
float unbiased_rate[2]; //Drehrate ohne Offset
PT1 yaw_rate = PT1(30, 1000); //zusätzlicher filter für Z gyro
const float rad_to_deg = 180 / 3.141592654;
float c = 0.0; //faktor cmeasure

//Input
float desired_angle[2];
float desired_yaw;
int16_t throttle;
int16_t min_throttle;

////////// PPM Input //////////
int channelAmount = 6;
PPMReader ppm(PPMInputPin, channelAmount); //Da vor main() definiert, laufen die Interrupts auf Core 0

////////// ESC Output //////////
ESC_Driver DriverVL(14);
ESC_Driver DriverVR(26);
ESC_Driver DriverHR(4);
ESC_Driver DriverHL(13);
// DSHOT_Driver DriverVL(14, DSHOT150);
// DSHOT_Driver DriverVR(26, DSHOT150);
// DSHOT_Driver DriverHR(4, DSHOT150);
// DSHOT_Driver DriverHL(13, DSHOT150);

////////// LSBs ////////////
#define LSBAcc 8192.0f
#define LSBGyr 32.8f

////////// Offsets ///////////////
const int16_t Off_Acc_X = 406;
const int16_t Off_Acc_Y = -106;
const int16_t Off_Acc_Z = 6785;
const int16_t Off_Gyr_X = 53;
const int16_t Off_Gyr_Y = -79;
const int16_t Off_Gyr_Z = -57;

const int16_t Off_Acc_Z_1g_compensated = Off_Acc_Z - (int16_t)LSBAcc;

int16_t throttle_offset[] = {0, 0, 0, 0}; //damit alle gleichzeitig starten

float pitch_offset = 0; //positiv nach vorne  //60
float roll_offset = 0; //positiv nach rechts //-80

void setup()
{
  Serial.begin(USB_BAUD); //USB ausgabe: Bei Processing Ausgabe 500000 nötig bei 1kHz Sampling

///////// Init Bluetooth Ausgabe /////////
#if defined(BLUETOOTH)
  Serial2.begin(230400);                                                                  //Bluetooth
  myqueue = xQueueCreate(20000, sizeof(float));                                           //Bluetooth
  xTaskCreatePinnedToCore(Bluetooth_Driver, "Bluetooth_Driver", 10000, NULL, 2, NULL, 0); //Bluetooth Priority 2
#endif                                                                                    // BLUETOOTH

///////// Init Spannungsmessung /////////
#if defined(SPANNUNGSMESSUNG)
  Init_Spannungsmessung();
#endif // SPANNUNGSMESSUNG

///////// Init GPIO Timing /////////
#if defined(GPIO_LOOP_TIMING)
  pinMode(GPIO_Output_Pin, OUTPUT);
#endif

  ///////// Init Summer /////////
  ledcSetup(7, 400, 8); //freier Channel 7, freq 400Hz, 8Bit auflösung
  ledcAttachPin(SummerPin, 7);
  ledcWrite(7, 0); //ausschalten

  ///////// Init MPU //////////////
  Wire.begin();          //Start I2C
  Wire.setClock(400000); //fast Mode

  Wire.beginTransmission(MPUaddress);
  Wire.write(0x6B);
  Wire.write(0); //Reset
  Wire.endTransmission(true);

  Wire.beginTransmission(MPUaddress);
  Wire.write(0x1B);
  Wire.write(2 << 3); //Gyro Sensitivity
  Wire.endTransmission(true);

  Wire.beginTransmission(MPUaddress);
  Wire.write(0x1C);
  Wire.write(1 << 3); //Acc Sensitivity
  Wire.endTransmission(true);

  Wire.beginTransmission(MPUaddress);
  Wire.write(0x1A); //DPLF
  Wire.write((3));  //0-6
  Wire.endTransmission(true);
  delay(50); //just wait

  //get min throttle damit Motoren sicher ausschalten
  min_throttle = get_min_throttle();

  //2 mal piepen für ready
  ledcWrite(7, 128); //50% duty
  delay(100);
  ledcWrite(7, 0);
  delay(100);
  ledcWrite(7, 128); //50% duty
  delay(100);
  ledcWrite(7, 0);

  ////////// start cyclic timer //////////
  Init_Timer();
}

void loop()
{

  while (loop_flag == 0) //1ms Loop warten
  {
  }
  loop_flag = 0; //flag rücksetzen

#if defined(GPIO_LOOP_TIMING)
  digitalWrite(GPIO_Output_Pin, HIGH); //Set Timing Pin HIGH at loop start
#endif

  //messStart = micros();

  ////////// Regelung //////////
  get_Input();

  //get_Lage();
  get_Lage_Kalman();

  PID();

  ////////// Ausgabe Processing //////////
  // Serial.print("Orientation: ");
  // Serial.print(0);
  // Serial.print(" ");
  // Serial.print(Total_angle[0]);
  // Serial.print(" ");
  // Serial.println(Total_angle[1]);

  ////////// Ausgabe Console //////////
  //Serial.println(testausgabe[1]);

  //////// Ausgabe Serial //////////
  if (Serial.available())
  {
    floatWriteSerial1(Total_angle[1]);
    floatWriteSerial1(desired_angle[1]);
    floatWriteSerial1(testausgabe[0]);  //pid roll
    floatWriteSerial1(unbiased_rate[1]);  
  }

  ////////// Bluetooth Ausgabe //////////
#if defined(BLUETOOTH)
  bluetoothCounter++; //Zähler incrementieren

  if (uxQueueSpacesAvailable(myqueue) < 3) //kein Platz mehr
  {
    sende = false;
    endesende = true;
  }

  if (bluetoothCounter >= BLUETOOTH_INTERVALL) //Nur senden wenn Intervall erreicht
  {
    if (Serial2.available() && sende) //Ausgabe wenn Bluetooth ready und sende Befehl
    {
      //xQueueSend(myqueue, &Total_angle[1], 1);      //float Wert1
      xQueueSend(myqueue, &bluetoothAusgabe[1], 1);    //soll drehrate x
      xQueueSend(myqueue, &bluetoothAusgabe[0], 1); //PId roll
      xQueueSend(myqueue, &bluetoothAusgabe[3], 1); //
      xQueueSend(myqueue, &bluetoothAusgabe[2], 1); //
    }

    bluetoothCounter = 0;
  }
#endif // BLUETOOTH

  //Serial.println(micros() - messStart);

#if defined(GPIO_LOOP_TIMING)
  digitalWrite(GPIO_Output_Pin, LOW); //Set Timing Pin low at loop end
#endif
}

////////// Funktionen //////////
void get_Input()
{
  //raw Werte in µs
  int32_t raw_angle[2];
  int32_t raw_yaw, raw_throttle, raw_aux;

  //umgerechnete Werte ohne Filter
  float Input_angle[2];
  float Input_yaw;
  float Input_aux;

  //Werte holen
  raw_throttle = ppm.latestValidChannelValue(3, 1000); // Standart 1000
  raw_angle[0] = ppm.latestValidChannelValue(2, 1500); //Standart 1500
  raw_angle[1] = ppm.latestValidChannelValue(1, 1500); //Standart 1500
  raw_yaw = ppm.latestValidChannelValue(4, 1500);      //Standart 1500
  raw_aux = ppm.latestValidChannelValue(5, 1500);      //Standart 1500

  //Werte umrechnen
  Input_angle[0] = map_long_float(raw_angle[0], 1000, 2000, Max_Input_angle, -Max_Input_angle);
  Input_angle[1] = map_long_float(raw_angle[1], 1000, 2000, Max_Input_angle, -Max_Input_angle);
  Input_yaw = (float)map(raw_yaw, 1000, 2000, Max_Input_yaw, -Max_Input_yaw);
  //Input_aux = (float)map(raw_aux, 1000, 2000, 1, 50) / 10.0; //für I Anteil
  //Input_aux = (float)map(raw_aux, 1000, 2000, 0, 50) / 100.0; //für D Anteil
  Input_aux = (float)map(raw_aux, 1000, 2000, 0, 60);

  //Filter

  throttle = raw_throttle - 1000 - min_throttle; //zunächst keinen Filter
  desired_angle[0] = 0.99 * desired_angle[0] + 0.01 * Input_angle[0];
  desired_angle[1] = 0.99 * desired_angle[1] + 0.01 * Input_angle[1];
  desired_yaw = Input_yaw; //zunächst keinen Filter

  //Begrenzung
  if (throttle > Max_Throttle)
  {
    throttle = Max_Throttle;
  }

  ////Aux Behandlung

  //kp_pitch = Input_aux;
  //kd_pitch = Input_aux;
  //ki_pitch = Input_aux;
  //kp_roll = Input_aux;
  //kd_roll = Input_aux;
  //ki_roll = Input_aux;
  //c = Input_aux;

#if defined(BLUETOOTH)
  Input_aux = (float)map(raw_aux, 1000, 2000, 1, 10) / 1.0; //Aux neu berechnen
  if (Input_aux > 1 && !endesende)
  {
    sende = true;
  }
  else if (Input_aux == 1)
  {
    endesende = false;
    sende = false;
    xQueueReset(myqueue);
  }

#endif // BLUETOOTH
}

void get_Lage_Kalman()
{
  //Erste Array Dimension 0:Pitch 1:Roll
  //Variablen
  int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;
  float newAngle[2];
  float y[2];    //Innovation
  float S[2];    //Innovation variance
  float K[2][2]; //Kalman Gain

  //Varianz Parameter
  const float Q_angle = 0.001f;      //process noise winkel
  const float Q_gyrobias = 0.00002f; //process noise gyro offset
  const float R_measure = 10.0f;     //measurement noise accelerometer

  //Zustand
  static float angle[2] = {0, 0}; //state angle 0:pitch 1:roll
  static float gyrobias[2] = {-Off_Gyr_Y / LSBGyr, Off_Gyr_X / LSBGyr};

  // error covariance matrix
  static float P[2][2][2] = {0};

  /*Reed the values that the accelerometre gives.*/
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)0x68, (uint8_t)6, true);

  Acc_rawX = Wire.read() << 8 | Wire.read(); //each value needs two registres
  Acc_rawY = Wire.read() << 8 | Wire.read();
  Acc_rawZ = Wire.read() << 8 | Wire.read();

  //Offsets
  Acc_rawX -= Off_Acc_X;
  Acc_rawY -= Off_Acc_Y;
  Acc_rawZ -= Off_Acc_Z_1g_compensated;

  /*Now we read the Gyro data in the same way as the Acc data. The adress for the
  * gyro data starts at 0x43.*/
  Wire.beginTransmission(0x68);
  Wire.write(0x43); //Gyro data first adress
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)0x68, (uint8_t)6, true);

  Gyr_rawX = Wire.read() << 8 | Wire.read(); //Once again we shift and sum
  Gyr_rawY = Wire.read() << 8 | Wire.read();
  Gyr_rawZ = Wire.read() << 8 | Wire.read();

  //Offset Z; X,Y im Filter implementiert
  Gyr_rawZ -= Off_Gyr_Z;
  // Drehrate Z
  bluetoothAusgabe[3] = yaw_rate.update(-Gyr_rawZ / LSBGyr);

  // Drehraten errechenen, Vorzeichen angepasst
  unbiased_rate[0] = -Gyr_rawY / LSBGyr - gyrobias[0];
  unbiased_rate[1] = Gyr_rawX / LSBGyr - gyrobias[1];

  // Winkel aus Accelerometer errechnen, Vorzeichen angepasst
  newAngle[1] = atanf((Acc_rawY / LSBAcc) / sqrtf((float)(Acc_rawX * Acc_rawX) / (LSBAcc * LSBAcc) + (float)(Acc_rawZ * Acc_rawZ) / (LSBAcc * LSBAcc))) * rad_to_deg;
  newAngle[0] = atanf(-1 * (Acc_rawX / LSBAcc) / sqrtf((float)(Acc_rawY * Acc_rawY) / (LSBAcc * LSBAcc) + (float)(Acc_rawZ * Acc_rawZ) / (LSBAcc * LSBAcc))) * -rad_to_deg;

  // Faktor für R_measure abhängig vom Input winkel
  float f_R_pitch = c * Total_angle[0] * Total_angle[0] + 1;

  //testausgabe[1] = f_R_pitch;
  //bluetoothAusgabe[2] = testPT1.update(newAngle[0]);
  //bluetoothAusgabe[1] = gyrobias[0];

  /* Kalman Algorithmus */
  // estimate of angle
  angle[0] += unbiased_rate[0] * Abtastzeit_s;
  angle[1] += unbiased_rate[1] * Abtastzeit_s;

  // estimate of error covariance matrix
  P[0][0][0] += Abtastzeit_s * (Abtastzeit_s * P[0][1][1] - P[0][0][1] - P[0][1][0] + Q_angle);
  P[0][0][1] -= Abtastzeit_s * P[0][1][1];
  P[0][1][0] -= Abtastzeit_s * P[0][1][1];
  P[0][1][1] += Q_gyrobias * Abtastzeit_s;

  P[1][0][0] += Abtastzeit_s * (Abtastzeit_s * P[1][1][1] - P[1][0][1] - P[1][1][0] + Q_angle);
  P[1][0][1] -= Abtastzeit_s * P[1][1][1];
  P[1][1][0] -= Abtastzeit_s * P[1][1][1];
  P[1][1][1] += Q_gyrobias * Abtastzeit_s;

  // Innovation berechnen
  y[0] = newAngle[0] - angle[0];
  y[1] = newAngle[1] - angle[1];

  // Innovation covariance berechnen
  S[0] = P[0][0][0] + R_measure * f_R_pitch;
  S[1] = P[1][0][0] + R_measure;

  //Kalman Gain berechnen
  K[0][0] = P[0][0][0] / S[0];
  K[0][1] = P[0][1][0] / S[0];

  K[1][0] = P[1][0][0] / S[1];
  K[1][1] = P[1][1][0] / S[1];

  // corrected angle,bias estimate
  angle[0] += K[0][0] * y[0];
  angle[1] += K[1][0] * y[1];

  gyrobias[0] += K[0][1] * y[0];
  gyrobias[1] += K[1][1] * y[1];

  // corrected error covariance matrix
  float P00_temp[2] = {P[0][0][0], P[1][0][0]}; //zwischenspeicher für berechnung
  float P01_temp[2] = {P[0][0][1], P[1][0][1]}; //zwischenspeicher für berechnung

  P[0][0][0] -= K[0][0] * P00_temp[0];
  P[0][0][1] -= K[0][0] * P01_temp[0];
  P[0][1][0] -= K[0][1] * P00_temp[0];
  P[0][1][1] -= K[0][1] * P01_temp[0];

  P[1][0][0] -= K[1][0] * P00_temp[1];
  P[1][0][1] -= K[1][0] * P01_temp[1];
  P[1][1][0] -= K[1][1] * P00_temp[1];
  P[1][1][1] -= K[1][1] * P01_temp[1];

  //Output an globale Variable
  Total_angle[0] = angle[0];
  Total_angle[1] = angle[1];
}

void PID()
{

  /*********  Variablen  **********/
  float sollDrehrate[3]; //X Y Z
  float PID_pitch, PID_roll, PID_yaw;

  int16_t throttle_output[4]; //von vorne links im Uhrzeigersinn

  /*********  Regler Algorithmus  **********/
  sollDrehrate[0] = angCtrlX.update(desired_angle[1], Total_angle[1]);
  sollDrehrate[1] = angCtrlY.update(desired_angle[0], Total_angle[0]);
  sollDrehrate[2] = desired_yaw;

  PID_roll = angVelCtrlX.update(sollDrehrate[0], unbiased_rate[1]);
  PID_pitch = angVelCtrlY.update(sollDrehrate[1], unbiased_rate[0]);
  PID_yaw = angVelCtrlZ.update(sollDrehrate[2], yaw_rate.get_Value());

  /////// PID Ausgabe //////

#if defined(BLUETOOTH)
  bluetoothAusgabe[0] = PID_yaw;
  bluetoothAusgabe[1] = sollDrehrate[2];
  bluetoothAusgabe[2] = angVelCtrlZ.getDebugValue();
#endif // BLUETOOTH

  testausgabe[0] = PID_roll;
  testausgabe[1] = angVelCtrlX.getDebugValue();

#if defined(NO_ESC_OUTPUT)
  throttle = 0;
#endif

  if (throttle < 40) //Gasstellung auf aus ?
  {
    for (int i = 0; i < 4; i++)
    {
      throttle_output[i] = 0; //alle Motoren aus
    }

    //Integral Anteile zurücksetzen
    angVelCtrlX.resetIntegral();
    angVelCtrlY.resetIntegral();
    angVelCtrlZ.resetIntegral();
  }

  else //Motoren sollen laufen

  {
    //Anpassung für HighRes PWM
    throttle *= 10;
    PID_pitch *= 10.0f;
    PID_roll *= 10.0f;
    PID_yaw *= 10.0f;

    //Anpassung für DSHOT Range
    // throttle *= 2;
    // PID_pitch *= 2.0f;
    // PID_roll *= 2.0f;
    // PID_yaw *= 2.0f;

    //throttle
    for (int i = 0; i < 4; i++)
    {
      throttle_output[i] = throttle; //Gasstellung
    }

    //PID Pitch
    PID_pitch -= pitch_offset;
    throttle_output[0] += (int16_t)PID_pitch;
    throttle_output[1] += (int16_t)PID_pitch;
    throttle_output[2] -= (int16_t)PID_pitch;
    throttle_output[3] -= (int16_t)PID_pitch;

    //PID Roll
    PID_roll += roll_offset;
    throttle_output[0] += (int16_t)PID_roll;
    throttle_output[1] -= (int16_t)PID_roll;
    throttle_output[2] -= (int16_t)PID_roll;
    throttle_output[3] += (int16_t)PID_roll;

    //PID yaw
    throttle_output[0] -= (int16_t)PID_yaw;
    throttle_output[1] += (int16_t)PID_yaw;
    throttle_output[2] -= (int16_t)PID_yaw;
    throttle_output[3] += (int16_t)PID_yaw;

    //Begrenzung
    for (int i = 0; i < 4; i++)
    {
      if (throttle_output[i] < (50 + throttle_offset[i]))
      {
        throttle_output[i] = 50 + throttle_offset[i];
      }
      if (throttle_output[i] > 10000)
      {
        throttle_output[i] = 10000;
      }
    }
  }

  //ESC Out PWM
  DriverVL.write_highres(throttle_output[0]);
  DriverVR.write_highres(throttle_output[1]);
  DriverHR.write_highres(throttle_output[2]);
  DriverHL.write_highres(throttle_output[3]);

  // DriverVL.write(throttle_output[0], false);
  // DriverVR.write(throttle_output[1], false);
  // DriverHR.write(throttle_output[2], false);
  // DriverHL.write(throttle_output[3], false);

  // DriverVL.write(100, false);
  // DriverVR.write(100, false);
  // DriverHR.write(100, false);
  // DriverHL.write(100, false);

  //Debug
  // for (int i = 0; i < 4; i++)
  // {
  //   Serial.print(throttle_output[i]);
  //   Serial.print(" : ");
  // }
  // Serial.println("");
}

int16_t get_min_throttle()
{
  int16_t maxWert = 0;
  int16_t newWert;
  for (int i = 0; i < 50; i++) //5sekunden
  {
    newWert = ppm.latestValidChannelValue(3, 1000); // Standart 1000
    if (newWert > maxWert)
    {
      maxWert = newWert;
    }
    delay(100);
  }

  if (maxWert < 1020)
  {
    return 1000;
  }
  else
  {
    return maxWert - 1000;
  }
}

void Init_Timer()
{
  //Timer init
  loop_timer = timerBegin(0, 80, true);                   //Timer 0, divider 80 für 1MHz, countup true
  timerAttachInterrupt(loop_timer, &loop_timerISR, true); //timer struct, isr, edge type?
  timerAlarmWrite(loop_timer, ABTASTZEIT_US, true);       //timer struct, CMP wert 1ms, autoreload
  timerAlarmEnable(loop_timer);                           //Interrupt enable
}

void IRAM_ATTR loop_timerISR()
{
  loop_flag = 1; //set flag
}

void floatWriteSerial1(float data)
{
  byte *b = (byte *)&data;
  Serial.write(b[0]);
  Serial.write(b[1]);
  Serial.write(b[2]);
  Serial.write(b[3]);
}

float map_long_float(int32_t x, int32_t in_min, int32_t in_max, float out_min, float out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

#if defined(BLUETOOTH)
void floatWrite(float data)
{
  byte *b = (byte *)&data;
  Serial2.write(b[0]);
  Serial2.write(b[1]);
  Serial2.write(b[2]);
  Serial2.write(b[3]);
}

void Bluetooth_Driver(void *parameter)
{
  float send_buffer;

  while (1)
  {
    if (uxQueueMessagesWaiting(myqueue))
    {
      xQueueReceive(myqueue, &send_buffer, 1);
      floatWrite(send_buffer);
      delay(1);
    }
    else
    {
      delay(1);
    }
  }

  vTaskDelete(NULL);
}
#endif // BLUETOOTH

#if defined(SPANNUNGSMESSUNG)
void Init_Spannungsmessung()
{
  //Init ADC
  analogSetAttenuation(ADC_11db);

  //Task Create
  xTaskCreatePinnedToCore(SpannungTest, "SpannungTest", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(Piepen, "Piepen", 10000, NULL, 1, &Piepen_handle, 0);
}

void SpannungTest(void *parameter)
{
  //Bewertung
  const float upperThreshold = 11; //V
  const float lowerThreshold = 8;  //V
  const float hysterese = 0.4;
  bool alarm = false;

  //Berechnung
  uint16_t raw;
  //float raw_filt = 0;
  float spannung = upperThreshold + hysterese; //Wert vorladen um piepen beim einschalten zu vermeiden
  const float factor = 0.005545;

  //Timing
  TickType_t LastWakeTime;
  const TickType_t Period = 100;
  LastWakeTime = xTaskGetTickCount();
  while (1)
  {
    vTaskDelayUntil(&LastWakeTime, Period); //alle Period aufwachen

    //Messen
    raw = analogRead(Spannung_Pin);
    //raw_filt = raw_filt * 0.9 + raw * 0.1;
    spannung = spannung * 0.9 + raw * factor * 0.1;
    //Serial.println(spannung); //debug

    //Bewerten
    if (spannung < upperThreshold && spannung > lowerThreshold)
    {
      //Alarm muss an sein
      if (!alarm)
      {
        xTaskNotify(Piepen_handle, 1, eSetValueWithOverwrite); //piep-Task mit 1 benachrichtigen
        alarm = true;
      }
    }
    else if ((spannung > (upperThreshold + hysterese)) || spannung < lowerThreshold)
    {
      //Alarm muss aus sein
      if (alarm)
      {
        xTaskNotify(Piepen_handle, 0, eSetValueWithOverwrite); //piep-Task mit 0 ausschalten
        alarm = false;
      }
    }
  }

  vTaskDelete(NULL);
}

void Piepen(void *parameter)
{
  uint32_t NotifyValue;
  while (1)
  {
    xTaskNotifyWait(0, 0, &NotifyValue, portMAX_DELAY); //Schlafen bis Notify

    while (NotifyValue > 0)
    {
      switch (NotifyValue)
      {
      case 1:              //langsam piepen
        ledcWrite(7, 128); //50% duty
        delay(500);
        ledcWrite(7, 0); //aus
        delay(500);
        ledcWrite(7, 128); //50% duty
        delay(500);
        ledcWrite(7, 0); //aus
        delay(500);
        break;
      }
      //schauen ob neue Notification ready ist, bei 0 zurück zum warten
      xTaskNotifyWait(0, 0, &NotifyValue, 0);
    }
  }

  vTaskDelete(NULL);
}

#endif // SPANNUNGSMESSUNG
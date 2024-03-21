#include <Arduino.h>
#include <CAN.h>
#include "qei.hpp"

#define PI 3.141592653589793

#define control_period 100 // 制御周期100µ秒

#define PIN_ENC_A 16 // ピン宣言
#define PIN_ENC_B 17
#define PIN_ENC_Z 18

#define PIN_ENABLE 23
#define PIN_PWM 25
#define PIN_MOTER_A 32
#define PIN_MOTER_B 33

int i = 0;
int16_t enccount = 0;

int packetSize = 0; // CAN関係の宣言
int receivedID = 0;
int myID = 100;

int dataReceived[9];
int dataTosend[9];

int ID_newID = 199;

uint32_t interval = 0;
uint32_t preinterval = 0;
uint32_t ccount = 0;
uint32_t jcount = 0;

float gain_kp = 0.0f;
float gain_ki = 0.0f;
float gain_kd = 0.0f;

float angle = 0.0f;
float omega = 0.0f;
short flg = 0;
short enableflg = 0;

void setup()
{
  Serial.begin(115200);
  qei_setup_x4(PCNT_UNIT_0, PIN_ENC_A, PIN_ENC_B);
  pinMode(PIN_ENC_Z, INPUT);

  pinMode(PIN_ENABLE, OUTPUT);
  pinMode(PIN_MOTER_A, OUTPUT);
  pinMode(PIN_MOTER_B, OUTPUT);

  ledcSetup(0, 12800, 8);
  ledcAttachPin(PIN_PWM, 0);

  CAN.setPins(26, 27);
  if (!CAN.begin(1000E3))
  {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }
}

double rotate(double D)
{
  if (D > 0)
  {
    digitalWrite(PIN_MOTER_A, HIGH);
    digitalWrite(PIN_MOTER_B, LOW);
    ledcWrite(0, (int)(D * 256));
  }
  else
  {
    digitalWrite(PIN_MOTER_A, LOW);
    digitalWrite(PIN_MOTER_B, HIGH);
    ledcWrite(0, (int)(-D * 256));
  }
  return 0;
}

void moterstop()
{
  digitalWrite(PIN_MOTER_A, LOW);
  digitalWrite(PIN_MOTER_B, LOW);
  ledcWrite(0, 0);
}

void moterbreak()
{
  digitalWrite(PIN_MOTER_A, HIGH);
  digitalWrite(PIN_MOTER_B, HIGH);
  ledcWrite(0, 0);
}

void loop()
{
  packetSize = CAN.parsePacket();
  if (packetSize || CAN.packetId() != -1)
  {
    receivedID = CAN.packetId();

    if (CAN.packetRtr())
    {
      if (packetSize != 0)
      {
        CAN.packetDlc();
      }
    }
    else
    {
      while (CAN.available())
      {
        if (packetSize != 0)
        {
          for (i = 0; i < packetSize; i++)
          {
            dataReceived[i] = CAN.read();
          }
        }
      }
    }

    if (receivedID == myID)
    {
    }

    if (receivedID == ID_newID)
    {
      myID = dataReceived[0];
    }
  }

  switch (flg)
  {
  case 0: // デフォではdisable
    enableflg = 0;
    break;

  case 1: // 位置制御モード
    break;

  case 2: // duty制御モード
    break;

  case 3: // 速度制御モード
    break;

  default: // 緊急停止
    moterstop();
    while (1)
    {
      Serial.println("emergency stoped please RESET");
      delay(1000);
    }
    break;
  }

  if (enableflg == 0)
  {
    digitalWrite(PIN_ENABLE, LOW);
  }
  else
  {
    digitalWrite(PIN_ENABLE, HIGH);
  }

  interval = micros() - preinterval;
  while (interval < control_period)
  {
    interval = micros() - preinterval;
  }
  preinterval = micros();
}

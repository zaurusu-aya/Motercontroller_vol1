// 3月21日作成

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
int myID = 0; // デフォでは0,newIDで0~7の範囲で変更可能

int dataReceived[9];
int dataTosend[9];

const int ID_newID = 199; // コマンドたち
const int ID_enable = 110;
const int ID_disable = 120;
const int ID_changeMode = 130;
const int ID_targetContorl = 140;
const int ID_originReset = 150;
const int ID_parameterSet = 160;

uint32_t interval = 0;
uint32_t preinterval = 0;
uint32_t ccount = 0;
uint32_t jcount = 0;

float gain_kp = 0.0f;
float gain_ki = 0.0f;
float gain_kd = 0.0f;
float gearRatio = 1.0f;

float targetAngle = 0.0f;
float targetOmega = 0.0f;

float angle = 0.0f;
float lastAngle = 0.0f;
float setAngle = 0.0f; // 位置制御の時のみ使う、指定された角度の変数
float omega = 0.0f;
float lastOmega = 0.0f;
float accelaration = 0.0f;
short flg = 0;
short enableflg = 0;
short mode = 0;
float duty = 0.0f;

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
  pcnt_get_counter_value(PCNT_UNIT_0, &enccount); // エンコーダパルス取得
  pcnt_counter_clear(PCNT_UNIT_0);

  lastAngle = angle; // 角度の計算
  angle += (enccount * (1.0 / 8192.0) * 2.0 * PI) / gearRatio;
  lastOmega = omega;
  omega = (lastAngle - angle) * (1000000.0 / control_period);
  accelaration = (lastOmega - omega) * (1000000.0 / control_period);

  packetSize = CAN.parsePacket(); // CANパケット受信開始
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

    if (receivedID == ID_enable + myID) // ID判別
    {
      flg = ID_enable;
      enableflg = 1;
    }

    if (receivedID == ID_disable + myID)
    {
      flg = ID_disable;
      enableflg = 0;
    }

    if (receivedID == ID_changeMode + myID)
    {
      flg = ID_changeMode;
      mode = dataReceived[0];
    }

    if (receivedID == ID_targetContorl + myID) // 受け取った目標値を反映
    {
      switch (mode)
      {
      case 1: // 角度制御モードの場合
        setAngle = (dataReceived[0] * 255 + dataReceived[1]) * (720.0 / 65535.0) - 360.0;
        targetOmega = (dataReceived[2] * 255 + dataReceived[3]) * ((200.0 * PI) / 65535.0);
        break;

      case 2: // 角速度制御モードの場合
        targetOmega = (dataReceived[0] * 255 + dataReceived[1]) * ((200.0 * PI) / 65535.0);
        if (dataReceived[2] == 1)
        {
          targetOmega = -targetOmega;
        }
        break;

      case 3: // duty制御モードの場合
        duty = dataReceived[1];
        if (dataReceived[0] == 0)
        {
          duty = -duty;
        }
        break;

      default:
        break;
      }
    }

    if (receivedID == ID_originReset + myID) // エンコーダの原点を設定
    {
      flg = ID_originReset;
    }

    if (receivedID == ID_parameterSet + myID) // 受け取ったパラメータを反映
    {
      flg = ID_parameterSet;

      gearRatio = (dataReceived[0] * 255 + dataReceived[1]) * (2000.0 / 65535.0);

      gain_kp = (dataReceived[2] * 255 + dataReceived[3]) * (2000.0 / 65535.0);

      gain_ki = (dataReceived[4] * 255 + dataReceived[5]) * (2000.0 / 65535.0);

      gain_kd = (dataReceived[6] * 255 + dataReceived[7]) * (2000.0 / 65535.0);
    }

    if (receivedID == ID_newID) // IDを変更
    {
      if (dataReceived[0] >= 0 && dataReceived[0] < 8)
      {
        myID = dataReceived[0];
      }
    }
  }

  switch (mode)
  {
  case 0: // デフォではdisable
    enableflg = 0;
    break;

  case 1: // 角度制御モード
    if (abs((setAngle - targetAngle) * (1000000.0 / control_period)) > targetOmega)
    {
      if ((setAngle - targetAngle) > 0)
      {
        targetAngle += targetOmega / (1000000.0 / control_period);
      }
      else
      {
        targetAngle -= targetOmega / (1000000.0 / control_period);
      }
    }
    else
    {
      targetAngle = setAngle;
    }
    duty = (targetAngle - angle) * gain_kp - (omega * gain_kd);

    if(duty > 1.0){
      duty = 1.0;
    }
    if(duty < -1.0){
      duty = -1.0;
    }

    rotate(duty);
    break;

  case 2: // 速度制御モード
    duty = (targetOmega - omega) * gain_kp - (accelaration * gain_kd);
    rotate(duty);
    break;

  case 3: // duty制御モード
    rotate(duty);
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

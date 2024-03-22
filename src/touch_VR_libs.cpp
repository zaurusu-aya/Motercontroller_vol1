#include <Arduino.h>
#include "touch_VR_libs.h"

void IKSolver::IKSolverSetup(float length1, float length2, float targetlimit_X0, float targetlimit_X1, float targetlimit_Y0, float targetlimit_Y1, float targetlimit_Z0, float targetlimit_Z1)
{
  l1 = length1;
  l2 = length2;
}

void IKSolver::IKSolve(float x, float y, float z)
{
  if (x == 0.0)
  { // 計算のバグを回避(予防)
    x = 0.001;
  }
  if (y == 0.0)
  {
    y = 0.001;
  }
  if (z == 0.0)
  {
    z = 0.001;
  }

  S = sqrt(pow(x, 2.0) + pow(z, 2.0)); //
  theta[0] = atan2(y, x) * (180.0 / PI);
  theta[1] = (acos((pow(S, 2.0) + pow(z, 2.0) + pow(l1, 2.0) - pow(l2, 2.0)) / (2 * l1 * sqrt(pow(S, 2) + pow(z, 2.0)))) + atan(z / S));
  theta[2] = (atan((z - l1 * sin(theta[1])) / (S - l1 * cos(theta[1])))) - theta[1]; // 逆運動学の計算たち
  theta[1] = theta[1] * (180 / PI);
  theta[2] = theta[2] * (180 / PI);
}

float IKSolver::get_theta0()
{
  return theta[0];
}

float IKSolver::get_theta1()
{
  return theta[1];
}

float IKSolver::get_theta2()
{
  return theta[2];
}

void PIDcontroll::PIDSetup(int ENABLEPIN, int PWMchannel_, float gear_Ratio, float duty_max)
{ // 各パラメータをメンバ変数に保存
  PIN_ENABLE = ENABLEPIN;
  PWMchannel = PWMchannel_;
  gearRatio = gear_Ratio;
  max_duty = duty_max;
}

void PIDcontroll::positioncontroll(float target_angle, int16_t ENCcount, float gain_kp, float gain_ki, float gain_kd)
{                                                                // ゲイン、目標角度、エンコーダーの生データを与えてPID制御
  currentAngle += ENCcount * (1.0 / (gearRatio * 8192.0)) * 360.0; // エンコーダーのパルスカウンタは制御周期毎にリセットすること
  deviation_sum += (currentAngle - target_angle);
  duty = (target_angle - currentAngle) * gain_kp - (currentAngle - lastAngle) * gain_kd - deviation_sum * gain_ki;
  lastAngle = currentAngle;

  if (duty > max_duty)
  {
    duty = max_duty;
  }
  if (duty < -max_duty)
  {
    duty = -max_duty;
  }
  if (duty > 1.0)
  {
    duty = 1.0;
  }
  if (duty < -1.0)
  {
    duty = -1.0;
  }
  int rotate = 0;
  if (duty > 0)
  {
    rotate = 0;
  }
  else
  {
    duty = -duty;
    rotate = 1;
  }
  digitalWrite(PIN_ENABLE, rotate);
  ledcWrite(PWMchannel, duty);
}

void PIDcontroll::rotate(float duty)
{
  if (duty > max_duty)
  {
    duty = max_duty;
  }
  if (duty < -max_duty)
  {
    duty = -max_duty;
  }
  if (duty > 1.0)
  {
    duty = 1.0;
  }
  if (duty < -1.0)
  {
    duty = -1.0;
  }
  int rotate = 0;
  if (duty > 0)
  {
    rotate = 0;
  }
  else
  {
    duty = -duty;
    rotate = 1;
  }
  digitalWrite(PIN_ENABLE, rotate);
  ledcWrite(PWMchannel, duty);
}
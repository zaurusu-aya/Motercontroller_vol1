#include "ropasu_libs.h"
#include <Arduino.h>

float sin_wave(float t, float ang_frq, float amplitude) {  //正弦波生成開始からの時間[s]、角周波数[rad/s]、振幅[mm]を入力→正弦波目標値生成
  float target_position = (amplitude / 2) * sin(t * ang_frq);
  return target_position;
}

void lowpass::lowpassSetup(float tau, float delta_T){ //時定数T[s],制御周期[s]を代入
  A = delta_T / (delta_T + (2.0 * tau));
  B = (delta_T - (2.0 * tau)) / (delta_T + (2.0 * tau));
  prev_X = 0.0;
  prev_Y = 0.0;
}

float lowpass::lowpassFiltering(float target_X) {  //入力Xを代入
  float Y = (A * target_X) + (A * prev_X) - (B * prev_Y);
  prev_X = target_X;
  prev_Y = Y;
  return Y;
}
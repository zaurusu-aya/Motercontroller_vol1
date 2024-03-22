#ifndef _ropasu_libs_h
#define _ropasu_libs_h

float sin_wave(float t, float ang_frq, float amplitude);

class lowpass
{
private:
  float prev_X;
  float prev_Y;
  float A;
  float B;

public:
  void lowpassSetup(float tau, float delta_T);
  float lowpassFiltering(float target_X);
};

#endif
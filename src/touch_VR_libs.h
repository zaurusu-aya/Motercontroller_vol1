#ifndef _touch_VR_libs_h
#define _touch_VR_libs_h

class IKSolver {
private:
  float l1;  //リンクの長さ
  float l2;
  float S;
  float theta[6];        //計算結果[degree]

public:
  void IKSolverSetup(float length1, float length2, float targetlimit_X0, float targetlimit_X1, float targetlimit_Y0, float targetlimit_Y1, float targetlimit_Z0, float targetlimit_Z1);
  void IKSolve(float x, float y, float z);
  float get_theta0();
  float get_theta1();
  float get_theta2();
};

class PIDcontroll
{
private:
  float max_duty;
  int PWMchannel;
  int PIN_ENABLE;
  int OFflg;
  int OFcount;
  double AXIS_angle;
  float gearRatio;
  double currentAngle;
  float duty;
  float omega;
  double lastAngle;
  double deviation_sum;

public:
  void PIDSetup(int ENABLEPIN, int PWMPIN, float gear_Ratio, float duty_max);
  void positioncontroll(float target_angle, int16_t ENCcount, float gain_kp, float gain_ki, float gain_kd);
  void rotate(float duty);
};

#endif
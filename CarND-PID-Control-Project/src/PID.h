#ifndef PID_H
#define PID_H
#include<vector>
#include<cmath>
#include<iostream>
using std::vector;
using std::cout;
using std::endl;

class PID {
  struct TwiddleState
  {
    int last_state=1;//1为增加 -1为减少
    int fail_times=0;//失败次数
  };
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  vector<double> p;
  double last_cte;
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  /*
  twiddle
  */
  vector<double> dp;
  bool Twiddle;
  int times;
  double error_sum;
  double best_error;
  int twiddle_times;
  vector<TwiddleState> pidState;
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double _Kp, double _Ki, double _Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */

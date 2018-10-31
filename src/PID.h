#ifndef PID_H
#define PID_H

#include <iostream>
#include <vector>
#include <numeric>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  std::vector<double> cte_record;
  std::vector<double> cte_record_average;

  double ave_error;
  double best_error;

  bool update_Kp_flag;
  bool update_Ki_flag;
  bool update_Kd_flag;

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
  void Init(double Kp, double Ki, double Kd);

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

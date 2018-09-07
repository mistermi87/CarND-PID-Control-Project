#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double total_error;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */

  // values for twiddle
    int twiddle_index=0;
    double twiddle_best_err=0;
    double p [3] = {0.165, 0.001, 2.6};
    double dp [3] ={0.00127093, 0, 0.0141215};
    bool worse=0;
    int twiddle_interval=1350;


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
  void TotalError(double cte);

  /*
  * Twiddle to find optimal value
  */
  void twiddle(int run);
};

#endif /* PID_H */

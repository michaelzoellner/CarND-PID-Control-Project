#ifndef PID_H
#define PID_H

#include <vector>

//using namespace std;

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
  
  // Memory for previous CTEs needed for calculation of derivative term and for tuning
  std::vector<double> prevCTE;
  int stepCountForAutoTune;
  int step;
  char paramToTune;
  float tuningFactor;
  double sumOfCTEs;


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
  
  // Function to auto-tune the parameters of the PID controller
  void AutoTune();
};

#endif /* PID_H */

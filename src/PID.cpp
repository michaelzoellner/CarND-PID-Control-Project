#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    prevCTE = 0.0;
    sumOfCTEs = 0.0;
}

void PID::UpdateError(double cte) {
    // This function calculates the output of each component of the PID controller
    sumOfCTEs += cte;
    
    // The P term is proportional to the error and hence only depends on the current CTE
    p_error = -Kp * cte;
    
    // The I term is calculated by integrating the error, hence previous result is kept
    i_error = -Ki * sumOfCTEs;
    
    // The D term is calculated using the derivative of 
    d_error = -Kd * (cte - prevCTE);
    
    prevCTE = cte;
}

double PID::TotalError() {
    cout << "PID errors are " << p_error << ", " << i_error << ", " << d_error << endl;
    return p_error + i_error + d_error;
}


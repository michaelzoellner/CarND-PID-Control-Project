#include "PID.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <numeric>

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {
}

PID::~PID() {
}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    sumOfCTEs = 0.0;
    prevCTE.push_back(0.0);
    stepCountForAutoTune = 1000;
    step = 0;
    paramToTune = 'p';
    tuningFactor = 1.1;
}

void PID::UpdateError(double cte) {
    // This function calculates the output of each component of the PID controller
    sumOfCTEs += cte;

    // The P term is proportional to the error and hence only depends on the current CTE
    p_error = -Kp * cte;

    // The I term is calculated by integrating the error, hence previous result is kept
    i_error = -Ki * sumOfCTEs;

    // The D term is calculated using the derivative of
    d_error = -Kd * (cte - prevCTE.back());
    //    cout << "errors are " << p_error << ", " << i_error << ", " << d_error << endl;

    prevCTE.push_back(cte);
    if (prevCTE.size() > 3 * stepCountForAutoTune)
        prevCTE.erase(prevCTE.begin());

    step++;
}

double PID::TotalError() {
    //    cout << "PID errors are " << p_error << ", " << i_error << ", " << d_error << endl;
    return p_error + i_error + d_error;
}

void PID::AutoTune() {
    /* 
     * Function to auto-tune the parameters of the PID controller 
     * The three parameters are tuned one at a time.
     * The mean squared CTE is assessed for x steps. This is done for the 
     * original, an increased and a decreased parameter value.
     * The new value is calculated through a parabola approach.
     */

    if (step > 0) {
        if (step % stepCountForAutoTune == 0) {
            switch (step / stepCountForAutoTune % 3) {
                case 1:
                {
                    switch (paramToTune) {
                        case 'p':
                        {
                            Kp *= tuningFactor;
                            break;
                        }
                        case 'i':
                        {
                            Ki *= tuningFactor;
                            break;
                        }
                        case 'd':
                        {
                            Kd *= tuningFactor;
                            break;
                        }
                    }
                    break;
                }
                case 2:
                {
                    switch (paramToTune) {
                        case 'p':
                        {
                            Kp *= pow(1 / tuningFactor, 2);
                            break;
                        }
                        case 'i':
                        {
                            Ki *= pow(1 / tuningFactor, 2);
                            break;
                        }
                        case 'd':
                        {
                            Kd *= pow(1 / tuningFactor, 2);
                            break;
                        }
                    }
                    break;
                }
                case 0:
                {
                    switch (paramToTune) {
                        case 'p':
                        {
                            Kp *= tuningFactor;
                            break;
                        }
                        case 'i':
                        {
                            Ki *= tuningFactor;
                            break;
                        }
                        case 'd':
                        {
                            Kd *= tuningFactor;
                            break;
                        }
                    }
                    // calculate the mean squared CTEs
                    double mse_ori = 0.0;
                    for (int ind = 0; ind < stepCountForAutoTune; ind++)
                        mse_ori += pow(prevCTE[ind], 2);
                    //                    cout << "mse_ori " << mse_ori << endl;
                    double mse_inc = 0.0;
                    for (int ind = stepCountForAutoTune; ind < 2 * stepCountForAutoTune; ind++)
                        mse_inc += pow(prevCTE[ind], 2);
                    //                    cout << "mse_inc " << mse_inc << endl;
                    double mse_dec = 0.0;
                    for (int ind = 2 * stepCountForAutoTune; ind < 3 * stepCountForAutoTune; ind++)
                        mse_dec += pow(prevCTE[ind], 2);
                    //                    cout << "mse_dec " << mse_dec << endl;

                    // calculate the minimum with a parabola approach
                    //                    (x2²(y3 - y1) - x1²(y3 - y2) - x3²(y2 - y1))/(2(x2(y3 - y1) - x1(y3 - y2) - x3(y2 - y1)))
                    float best = (pow(tuningFactor, 2)*(mse_dec - mse_ori) - (mse_dec - mse_inc) - pow(1 / tuningFactor, 2)*(mse_inc - mse_ori)) / (2 * (tuningFactor * (mse_dec - mse_ori)-(mse_dec - mse_inc) - 1 / tuningFactor * (mse_inc - mse_ori)));
                    cout << "Best factor for K" << paramToTune << " is " << best << endl;
                    best = max(best, 1 / tuningFactor);
                    //                    cout << "Best after max " << best << endl;

                    best = min(best, tuningFactor);
                    //                    cout << "Best after min " << best << endl;
                    if (!isnan(best)) {
                        switch (paramToTune) {
                            case 'p':
                            {
                                Kp *= best;
                                paramToTune = 'i';
                                break;
                            }
                            case 'i':
                            {
                                Ki *= best;
                                paramToTune = 'd';
                                break;
                            }
                            case 'd':
                            {
                                Kd *= best;
                                paramToTune = 'p';
                                break;
                            }
                        }
                        cout << "New set of parameters is Kp=" << Kp << ", Ki=" << Ki << ", Kd=" << Kd << endl;
                    } else
                        cout << "Auto-tuning for K" << paramToTune << " resulted in NaN, repeating... " << endl;
                }
            }
        }
    }
}
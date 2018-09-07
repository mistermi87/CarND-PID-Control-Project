#include "PID.h"
#include <math.h>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_in, double Ki_in, double Kd_in) {
    Kp=Kp_in;
    Ki=Ki_in;
    Kd=Kd_in;
    p_error=0;
    i_error=0;
    d_error=0;
    total_error=0;
}

void PID::UpdateError(double cte) {
    d_error=(cte-p_error); //whereas p_error has the value of previous cte;
    i_error=i_error+cte;
    p_error=cte;
}

void PID::TotalError(double cte) {
    //since I am trying to implement a "learning while driving" approach the total error is just a moving average of the last lap's absolute distances to the optimum
    total_error=(1-1.0/(twiddle_interval))*total_error+(1.0/(twiddle_interval))*fabs(cte);
}

void PID::twiddle(int run){

    if(run%twiddle_interval==0){
        if(worse==0){
            if(twiddle_index==3) twiddle_index=0;
            if(twiddle_index==1) twiddle_index=2;

            if(total_error < twiddle_best_err){
                twiddle_best_err = total_error;
                dp[twiddle_index] *= 1.1;
                p[twiddle_index] += dp[twiddle_index];
                Kp=p[0];
                Ki=p[1];
                Kd=p[2];
                worse=0;
                twiddle_index+=1;
            }else{
                p[twiddle_index] -= 2 * dp[twiddle_index];
                Kp=p[0];
                Ki=p[1];
                Kd=p[2];
                worse=1;
            }

        }else{ //-->if worse==1
            if(total_error < twiddle_best_err){
                twiddle_best_err = total_error;
                dp[twiddle_index] *= 1.1;
                p[twiddle_index] += dp[twiddle_index];
                Kp=p[0];
                Ki=p[1];
                Kd=p[2];
                worse=0;
                twiddle_index+=1;
            }else{
                p[twiddle_index] += dp[twiddle_index];
                dp[twiddle_index] *= 0.9;
                Kp=p[0];
                Ki=p[1];
                Kd=p[2];
                worse=0;
                twiddle_index+=1;
                }
    }
    cout<< "---Iteration: " << run << " TI: " << twiddle_index <<" P:" << p[0] << " "<< p[1] << " "<< p[2] << " dp: "<< dp[0] << " "<< dp[1] << " "<< dp[2]<<endl;
    }
}


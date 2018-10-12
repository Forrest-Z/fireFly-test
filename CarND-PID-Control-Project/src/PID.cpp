#include "PID.h"

using namespace std;
#define twiddle_loop 100
/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}
int last_sign;
void PID::Init(double _Kp, double _Ki, double _Kd) {
    p.resize(3);
    p={_Kp,_Ki,_Kd};
    /*Kp=_Kp;
    Ki=_Ki;
    Kd=_Kd;*/
    p_error=i_error=d_error=0;
    last_cte=0;
    dp.resize(3,0.001);
    Twiddle=true;
    times=0;
    error_sum=0;
    best_error=__INT_MAX__;
    twiddle_times=0;
    last_sign=0;
    pidState.resize(3);
}

void PID::UpdateError(double cte) {
    p_error=cte;
    d_error=cte-last_cte;
    i_error+=cte;
    last_cte=cte;
    times++;

    if(Twiddle)
    {
        error_sum+=abs(cte);
        if(times%twiddle_loop==0 )
        {
            error_sum/=twiddle_loop;
            if(error_sum<best_error)
            {
                best_error=error_sum;
                pidState[twiddle_times].fail_times=0;
                dp[twiddle_times]*=1.05;
                p[twiddle_times]+=pidState[twiddle_times].last_state*dp[twiddle_times];
            }
            else if(pidState[twiddle_times].fail_times<2)
            {
                pidState[twiddle_times].last_state*=-1;
                p[twiddle_times]+=2*pidState[twiddle_times].last_state*dp[twiddle_times];
                pidState[twiddle_times].fail_times++;
            }
            else
            {
                pidState[twiddle_times].fail_times=0;
                pidState[twiddle_times].last_state*=-1;
                p[twiddle_times]+=pidState[twiddle_times].last_state*dp[twiddle_times];
                dp[twiddle_times]*=0.95;
                twiddle_times++;
                twiddle_times%=3;
            }
        }
        cout<<"P:"<<p[0]<<" I:"<<p[1]<<" D:"<<p[2]<<" "<<error_sum<<endl;
        error_sum=0;
    }
}

double PID::TotalError() {
    return p[0]*p_error+p[2]*d_error/*+p[1]*i_error*/;
}


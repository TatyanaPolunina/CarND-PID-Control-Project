#include "PID.h"
#include <iostream>

PID::PID()
    : p_error(0)
    , i_error(0)
    , d_error(0)
{}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, int num_iter_to_tune) {

    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    this->num_iter_to_tune = num_iter_to_tune;
    common_error = 0;
}

void PID::UpdateError(double cte) {
    ++num_iter;
    i_error += cte;
    d_error = cte - p_error;
    p_error = cte;
    total_i += abs(cte);
    common_error += std::abs(p_error);
    //exit if don't need to tune params
    if (num_iter % num_iter_to_tune != 0)
    {
        return;
    }

    common_error = common_error / num_iter;
    if (common_error > tollerance)
    {
        double diff_error = prev_error - common_error;
        //use -derivatives for backpropagation
        TuneParams(p_error, total_i, d_error, diff_error * grad_delta);
        std::cout << "K_params " << Kp << " " << Ki << " " << Kd << std::endl;
        std::cout << "common_error " << common_error << std::endl;
        prev_error = common_error;
        //clean up for the next iteration
        common_error = 0;
        num_iter = 0;
        total_i = 0;
    }
}

double PID::TotalError() {
    return -Kp * p_error - Ki * i_error - Kd * d_error;
}

void PID::TuneParams(double change_p, double change_i, double change_d, double delta)
{
    Kp += Kp * change_p * delta;
    Ki += Ki * change_i * delta;
    Kd += Kd * change_d * delta;
}

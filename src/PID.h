#ifndef PID_H
#define PID_H

#include <vector>
class PID {
public:
    /**
    * Constructor
    */
    PID();

    /**
    * Destructor.
    */
    virtual ~PID();

    /**
    * Initialize PID.
    * @param (Kp_, Ki_, Kd_) The initial PID coefficients
    */
    void Init(double Kp_, double Ki_, double Kd_, int num_iter_to_tune);

    /**
    * Update the PID error variables given cross track error.
    * @param cte The current cross track error
    */
    void UpdateError(double cte);

    /**
    * Calculate the total PID error.
    * @output The total PID error
    */
    double TotalError();

private:
    /**
    tune the PID K parameters using backpropagation approach
    */
    void TuneParams(double change_p, double change_i, double change_d, double delta);
    /**
    * PID Errors
    */
    double p_error;
    double i_error;
    double d_error;

    /**
    * PID Coefficients
    */
    double Kp;
    double Ki;
    double Kd;
    double tollerance = 0.001;
    double grad_delta = 0.01;
    int num_iter_to_tune;
    int num_iter = 0;
    double common_error = 0;
    double prev_error = 0;
    double total_i = 0;
    double best_err = 1000;
};

#endif  // PID_H
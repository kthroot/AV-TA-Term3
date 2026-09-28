#pragma once
// Implement Controller so that, given only the target angle, the last
// measured angle, and the timestep, it drives the system to the target --
// despite whatever nonlinearity you identified from the CSVs.
//
// This is the file you submit. You can add private members, helper methods,
// filters, whatever your design needs. We will never run your internals.

#include <algorithm>
#include "controller_interface.hpp"

class Controller : public IController {

private:
    double kp = 3.0;
    double ki = 0.5;
    double kd = 0.03;

    double integral = 0.0;
    double previous_measured = 0.0;

    double max_command = 15.0;

    double integral_limit = 10.0;

    bool first_update = true;


public:

    double update(double target, double measured ,double dt) override
    {
        if (dt <= 0.0) {
            return 0.0;
        }


        double error = target - measured;
        double derivative = 0.0;

        if (!first_update) {
            derivative = -(measured - previous_measured)/dt;
        }

        double p_term = kp * error;
        double d_term = kd * derivative;

        double command_without_integral =
            p_term + d_term + ki * integral;

        bool saturated_high =
            command_without_integral >= max_command;

        bool saturated_low =
            command_without_integral <= -max_command;

        bool integrate =
            !saturated_high &&
            !saturated_low;


        if (saturated_high && error > 0.0) {
            integrate = false;
        }

        if (saturated_low && error < 0.0) {
            integrate = false;
        }


        if (integrate) {

            integral += error * dt;

            integral = std::clamp(integral,-integral_limit,integral_limit);
        }

        double command = kp * error + ki * integral + kd * derivative;

        command = std::clamp(command,-max_command,max_command);

        previous_measured = measured;
        first_update = false;

        return command;
    }


    void reset() override
    {
        integral = 0.0;
        previous_measured = 0.0;
        first_update = true;
    }
};
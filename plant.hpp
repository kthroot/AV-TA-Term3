#pragma once
// Your model of the actuator, reconstructed from the decoded CSVs. This is
// the Part B deliverable, alongside your written notes.
//
// Implement step(): given a commanded velocity and a timestep, return the
// measured output angle. The placeholder below is a bare integrator with
// gain 1 -- NOT the real actuator. Replace it with what the data shows
// (dynamics, gain, any nonlinearity, any lag), or the harness proves nothing.

#include <cmath>

struct Plant {
    static constexpr double m = 1.30;       // deg/s per command unit
    static constexpr double tau = 0.067;    // seconds
    static constexpr double cost = 2.5; // degrees

    double Mangle = 0.0;
    double Mvelo = 0.0;

    double OPangle = 0.0;
    double angle = 0.0;

    double step(double u_cmd, double dt) {

        const double desired_velocity = m * u_cmd;
        const double alpha = std::exp(-dt / tau);

        Mvelo = desired_velocity + (Mvelo - desired_velocity) * alpha;
        Mangle += Mvelo * dt;

        if (Mangle > OPangle + cost) {
            OPangle = Mangle - cost;

        }
        else if (Mangle < OPangle - cost) {
            OPangle = Mangle + cost;
        }

        const double measured =
            std::round(OPangle/0.1) * 0.1;

        return measured;
    }

    void reset() { 
        Mangle = 0.0;
        Mvelo = 0.0;
        OPangle = 0.0;
    }
};

#pragma once

#include <algorithm>

// Generic PID controller for scalar signals.
class PID
{
public:
    PID(double kp = 0.0, double ki = 0.0, double kd = 0.0,
        double minOutput = -1e9, double maxOutput = 1e9)
        : kp(kp), ki(ki), kd(kd),
          minOutput(minOutput), maxOutput(maxOutput)
    {}

    void setGains(double p, double i, double d)
    {
        kp = p;
        ki = i;
        kd = d;
    }

    void setOutputLimits(double minOut, double maxOut)
    {
        minOutput = minOut;
        maxOutput = maxOut;
    }

    void reset()
    {
        integral = 0.0;
        prevError = 0.0;
        firstUpdate = true;
    }

    // Compute control output for the given error and timestep.
    double update(double error, double dt)
    {
        if (dt <= 0.0)
            return 0.0;

        if (firstUpdate)
        {
            prevError = error;
            firstUpdate = false;
        }

        integral += error * dt;
        double derivative = (error - prevError) / dt;
        prevError = error;

        double output = kp * error + ki * integral + kd * derivative;

        output = std::clamp(output, minOutput, maxOutput);
        return output;
    }

private:
    double kp;
    double ki;
    double kd;

    double minOutput;
    double maxOutput;

    double integral = 0.0;
    double prevError = 0.0;
    bool firstUpdate = true;
};



#pragma once

#include "ODESystem.h"
#include <string>

namespace nereid
{
    class DIPC : public ODESystem
    {
    public:
        struct Params
        {
            double m0;  // mass of cart
            double m1;  // mass of first pendulum (attached to cart)
            double m2;  // mass of second pendulum
            double L1;  // length of first pendulum
            double L2;  // length of second pendulum
        };

        void setParams(const Params& params);

        State f(const State& x) const;

        static std::string to_json(const State& x);

    private:
        // primary parameters
        Params p_;

        // derived constants
        void computeConstants(void);
        double d1_;
        double d2_;
        double d3_;
        double d4_;
        double d5_;
        double d6_;
        double f1_;
        double f2_;
    };
}
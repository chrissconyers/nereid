
#pragma once

#include "ODESystem.h"
#include <string>

namespace nereid
{
    class DIPC : public ODESystem
    {
    public:
        State f(const State& x) const;

        static std::string to_json(const State& x);

    private:
        double m0_;  // mass of cart
        double m1_;  // mass of first pendulum (attached to cart)
        double m2_;  // mass of second pendulum
        double L1_;  // length of first pendulum
        double L2_;  // length of second pendulum
    };
}
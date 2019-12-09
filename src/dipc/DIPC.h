
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

            static std::string to_json(const Params& params);
            static Params from_json(const std::string json_str);
        };

        void setParams(const Params& params);

        State f(const State& x, const Input& u) const;

        static std::string state_to_json(const State& x);
        static State state_from_json(const std::string json_str);
        static std::string input_to_json(const Input& u);
        static Input input_from_json(const std::string json_str);

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
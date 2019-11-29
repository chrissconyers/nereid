
#pragma once

#include "dynamics_defs.h"

#include <Eigen/Core>

namespace nereid
{
    class ODESystem
    {
    public:
        virtual ~ODESystem(void) {}
        virtual State f(const State& x, const Input& u) const = 0;
    };
}

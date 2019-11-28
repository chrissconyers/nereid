
#pragma once

#include "ODESystem.h"

namespace nereid
{
    class DIPC : public ODESystem
    {
    public:
        State f(const State& x);
    };
}
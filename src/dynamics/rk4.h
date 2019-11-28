
#pragma once

#include "ODESystem.h"


namespace nereid
{
namespace propagation
{
    State rk4(const State& x, float dt, const ODESystem& system);
}
}

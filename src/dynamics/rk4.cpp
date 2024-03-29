
#include "rk4.h"

namespace nereid
{

State propagation::rk4(const State& x, const Input& u, float dt, const ODESystem& system)
{
    State k1 = dt * system.f(x, u);
    State k2 = dt * system.f(x + 0.5*k1, u);
    State k3 = dt * system.f(x + 0.5*k2, u);
    State k4 = dt * system.f(x + k3, u);
    return x + (k1 + 2*k2 + 2*k3 + k4)/6.0;
}

}

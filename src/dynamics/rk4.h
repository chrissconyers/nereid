
#pragma once

#include "ODESystem.h"

#include <memory>


namespace nereid
{
namespace propagation
{
    State rk4(const State& x, float dt, std::shared_ptr<ODESystem> system);
}
}

#if 0
namespace propagation
{
    template

    template<class T, class K>
    Eigen::VectorXf rk4(const Eigen::VectorXf& state, float dt, const T& params, K& dynamics)
    {
        Eigen::VectorXf x1, x2, x3, x4;
            
        x1 = dt * dynamics(state, params);
        x2 = dt * dynamics(state + .5*x1, params);    
        x3 = dt * dynamics(state + .5*x2, params);
        x4 = dt * dynamics(state + x3, params);

        return state + (x1 + 2*x2 + 2*x3 + x4)/6.;
    }
}
#endif

#if 0
#include "Eigen/Dense"
namespace propagation
{
    
template<class T, class K>
Eigen::VectorXf rk4(const Eigen::VectorXf& state, float dt, const T& params,
 K& dynamics)
{
    Eigen::VectorXf x1, x2, x3, x4;
        
    x1 = dt * dynamics(state, params);
    x2 = dt * dynamics(state + .5*x1, params);    
    x3 = dt * dynamics(state + .5*x2, params);
    x4 = dt * dynamics(state + x3, params);

    return state + (x1 + 2*x2 + 2*x3 + x4)/6.;
}

template<class T, class K>
Eigen::VectorXf forwardEuler(const Eigen::VectorXf& state, float dt, const T& params,
 K& dynamics)
{
    return state + dt*dynamics(state, params);
}

template<class T, class K>
Eigen::VectorXf forwardEulerTen(const Eigen::VectorXf& state, float dt, const T& params,
 K& dynamics)
{
    Eigen::VectorXf state_k = state;
    for(int i =0; i< 10; i++){
        state_k += .1*dt*dynamics(state_k, params);
    }
    return state_k;
}

}
#endif
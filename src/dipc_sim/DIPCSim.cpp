

#include "DIPCSim.h"

#include "rk4.h"


namespace nereid
{

class DIPCSim::PrivateImpl
{
public:
    DIPC dipc;
    State state;
};


DIPCSim::DIPCSim(void)
: impl_(std::make_unique<PrivateImpl>())
{
}

DIPCSim::~DIPCSim(void)
{
}

void DIPCSim::init(double x, double theta_1, double theta_2, const DIPC::Params& params)
{
    impl_->state = (Eigen::VectorXd(6) << x, 0.0, theta_1, 0.0, theta_2, 0.0).finished();
    impl_->dipc.setParams(params);
}

void DIPCSim::tick(double dt)
{
    State new_state = propagation::rk4(impl_->state, dt, impl_->dipc);
    impl_->state = new_state;
}

std::string DIPCSim::stateStr(void)
{
    return DIPC::to_json(impl_->state);
}

}

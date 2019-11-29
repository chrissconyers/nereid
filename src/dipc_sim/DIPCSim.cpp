

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

void DIPCSim::init(double theta_0, double theta_1, double theta_2, const DIPC::Params& params)
{
    impl_->state = (Eigen::VectorXd(6) << theta_0, theta_1, theta_2, 0.0, 0.0, 0.0).finished();
    impl_->dipc.setParams(params);
}

void DIPCSim::tick(double dt)
{
    Input input = (Eigen::VectorXd(1) << 0.0).finished();
    State new_state = propagation::rk4(impl_->state, input, dt, impl_->dipc);
    impl_->state = new_state;
}

std::string DIPCSim::stateStr(void)
{
    return DIPC::to_json(impl_->state);
}

}

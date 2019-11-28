
#include "DIPC.h"

#include "constants.h"
#include <nlohmann/json.hpp>


namespace nereid
{

void DIPC::setParams(const Params& params)
{
    p_ = params;
    computeConstants();
}

State DIPC::f(const State& x) const
{
    auto x_dot = x;
    return x_dot;
}

// I'm sure there's a better way to do this
std::string DIPC::to_json(const State& x)
{
    nlohmann::json json;
    json["x"] = {};
    json["x"].push_back(x(0));
    json["x"].push_back(x(1));
    json["x"].push_back(x(2));
    json["x"].push_back(x(3));
    json["x"].push_back(x(4));
    json["x"].push_back(x(5));
    return json.dump();
}

void DIPC::computeConstants(void)
{
    d1_ = p_.m0 + p_.m1 + p_.m2;
    d2_ = (0.5*p_.m1 + p_.m2)*p_.L1;
    d3_ = 0.5*p_.m2*p_.L2;
    d4_ = (p_.m1/3.0 + p_.m2)*p_.L1*p_.L1;
    d5_ = 0.5*p_.m2*p_.L1*p_.L2;
    d6_ = p_.m2*p_.L2*p_.L2/3.0;
    f1_ = (0.5*p_.m1 + p_.m2)*p_.L1*G;
    f2_ = 0.5*p_.m2*p_.L2*G;
}

}

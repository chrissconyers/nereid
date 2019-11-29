
#include "DIPC.h"

#include "constants.h"
#include <nlohmann/json.hpp>
#include <Eigen/LU>


namespace nereid
{

void DIPC::setParams(const Params& params)
{
    p_ = params;
    computeConstants();
}

State DIPC::f(const State& x, const Input& u) const
{
    // x(0) = theta_0     : position of cart
    // x(1) = theta_1     : angle of first pendulum
    // x(2) = theta_2     : angle of second pendulum
    // x(3) = theta_0_dot : velocity of cart
    // x(4) = theta_1_dot : angular velocity of first pendulum
    // x(5) = theta_2_dot : angular velocity of second pendulum

    Eigen::Matrix3d D;
    D << d1_, d2_*cos(x(1)), d3_*cos(x(2)),
         d2_*cos(x(1)), d4_, d5_*cos(x(1)-x(2)),
         d3_*cos(x(2)), d5_*cos(x(1)-x(2)), d6_;

    Eigen::Matrix3d C;
    C << 0, -d2_*sin(x(1))*x(4), -d3_*sin(x(2))*x(5),
         0, 0, d5_*sin(x(1)-x(2))*x(5),
         0, -d5_*sin(x(1)-x(2))*x(4), 0;

    Eigen::Vector3d G;
    G << 0, -f1_*sin(x(1)), -f2_*sin(x(2));

    Eigen::Vector3d H;
    H << 1, 0, 0;

    Eigen::Matrix3d D_inv;
    D_inv = D.inverse();

    Eigen::MatrixXd A1 = Eigen::MatrixXd::Zero(6,6);
    A1.topRightCorner(3,3) = Eigen::Matrix3d::Identity();
    A1.bottomRightCorner(3,3) = -D_inv*C;
    //A1()

    Eigen::VectorXd A2 = Eigen::VectorXd::Zero(6);
    A2.tail(3) = -D_inv*G;

    Eigen::VectorXd A3 = Eigen::VectorXd::Zero(6);
    A3.tail(3) = -D_inv*H;

    return A1*x + A2 + A3*u;
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
    f1_ = (0.5*p_.m1 + p_.m2)*p_.L1*Constants::g;
    f2_ = 0.5*p_.m2*p_.L2*Constants::g;
}

}

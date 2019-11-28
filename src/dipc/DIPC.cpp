
#include "DIPC.h"

#include <nlohmann/json.hpp>

namespace nereid
{

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

}



#include "DIPCSim.h"

#include "DIPC.h"
#include "rk4.h"

#include <fmt/core.h>
#include <nlohmann/json.hpp>


namespace nereid
{

class DIPCSim::PrivateImpl
{
public:
    DIPC dipc;

    PrivateImpl(void)
    {
    }
};


DIPCSim::DIPCSim(void)
: impl_(std::make_unique<PrivateImpl>())
{
}

DIPCSim::~DIPCSim(void)
{
}

std::string DIPCSim::publishState(void)
{
    static int count = 0;

    
    nlohmann::json j;
    j["name"] = "tarquin";
    j["id"] = ++count;
    std::string msg_str = j.dump();

    //rclcpp::Clock clock;
    //auto x = clock.now();
    //fmt::print("time: {} seconds\n", x.seconds());

    fmt::print("publishing '{}'\n", msg_str);
    return msg_str;
}

}

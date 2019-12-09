
#pragma once

#include "DIPC.h"

#include <memory>
#include <string>


namespace nereid
{
    class DIPCSim
    {
    public:
        DIPCSim(void);
        ~DIPCSim(void);

        void init(double x, double theta_1, double theta_2, const DIPC::Params& params);
        void tick(double dt);
        void setInput(const Input& input);

        std::string stateStr(void);

        std::string publishState(void);

    private:
        class PrivateImpl;
        std::unique_ptr<PrivateImpl> impl_;
    };
};

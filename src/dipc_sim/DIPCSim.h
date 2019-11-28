
#pragma once

#include <memory>
#include <string>


namespace nereid
{
    class DIPCSim
    {
    public:
        DIPCSim(void);
        ~DIPCSim(void);

        std::string publishState(void);

    private:
        class PrivateImpl;
        std::unique_ptr<PrivateImpl> impl_;
    };
};
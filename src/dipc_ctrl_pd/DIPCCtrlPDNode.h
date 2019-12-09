
#include "rclcpp/rclcpp.hpp"


namespace nereid
{

class DIPCCtrlPDNode : public rclcpp::Node
{
public:
    DIPCCtrlPDNode(void);
    ~DIPCCtrlPDNode(void);

private:
    class PrivateImpl;
    std::unique_ptr<PrivateImpl> impl_;

    void init(void);
};

}

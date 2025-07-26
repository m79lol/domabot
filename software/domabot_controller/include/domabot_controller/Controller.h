#ifndef Domabot_Controller_h
#define Domabot_Controller_h

#include <domabot_interfaces/msg/controller_status.hpp>

#include <rclcpp/rclcpp.hpp>

#include <modbus.h>

#include <mutex>

namespace Domabot {

class Controller : public rclcpp::Node {
  public:
    using CnstPtr = std::shared_ptr<const Controller>;
    using Ptr = std::shared_ptr<Controller>;

  protected:
    modbus_t* m_cntx = nullptr;
    mutable std::mutex m_mtx;
    bool m_isConnected = false;

    rclcpp::Publisher<domabot_interfaces::msg::ControllerStatus>::SharedPtr m_statusPublisher;
    rclcpp::TimerBase::SharedPtr m_statusTimer;

    void runModbusOperation(
      std::function<bool (modbus_t*)> operation
    );

    void statusTimerCallback();

    void getControllerData();

  public:
    Controller();

    Controller(const Controller& other)            = delete;
    Controller(Controller&& other)                 = delete;
    Controller& operator=(const Controller& other) = delete;
    Controller& operator=(Controller&& other)      = delete;

    virtual ~Controller() noexcept;



}; // Controller

} // Domabot

#endif // Domabot_Controller_h
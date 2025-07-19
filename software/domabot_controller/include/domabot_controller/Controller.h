#ifndef domabot_controller_h
#define domabot_controller_h

#include <domabot_interfaces/msg/controller_status.hpp>

#include <rclcpp/rclcpp.hpp>

#include <modbus.h>

#include <mutex>

namespace Domabot {

class Controller {
  protected:
    struct ParamNames {
      static const std::string m_baudRate;
      static const std::string m_controllerPath;
      static const std::string m_dataBits;
      static const std::string m_parity;
      static const std::string m_stopBits;
    };

    const rclcpp::Node::SharedPtr m_node = nullptr;
    modbus_t* m_cntx = nullptr;
    mutable std::mutex m_mtx;

    rclcpp::Publisher<domabot_interfaces::msg::ControllerStatus>::SharedPtr m_statusPublisher;
    rclcpp::TimerBase::SharedPtr m_statusTimer;

  public:
    Controller(
      const rclcpp::Node::SharedPtr node
    );

    Controller(const Controller& other)            = delete;
    Controller(Controller&& other)                 = delete;
    Controller& operator=(const Controller& other) = delete;
    Controller& operator=(Controller&& other)      = delete;

    virtual ~Controller() noexcept;

    void statusTimerCallback();

}; // Controller

} // Domabot

#endif
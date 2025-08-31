/**
 * @file Core.h
 * @brief Domabot Core class header file.
 * @copyright Copyright 2025 m79lol
*/
#ifndef DOMABOT_CORE__CORE_H_
#define DOMABOT_CORE__CORE_H_

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace Domabot {

/** @brief Class node for core logic application of Domabot robot. */
class Core : public rclcpp::Node {
  public:
    using Ptr = std::shared_ptr<Core>;
    using CnstPtr = std::shared_ptr<const Core>;

  public:
    Core();

    Core(const Core& other)            = delete;
    Core(Core&& other)                 = delete;
    Core& operator=(const Core& other) = delete;
    Core& operator=(Core&& other)      = delete;

    virtual ~Core() = default;
};  // Core

}  // namespace Domabot

#endif  // DOMABOT_CORE__CORE_H_

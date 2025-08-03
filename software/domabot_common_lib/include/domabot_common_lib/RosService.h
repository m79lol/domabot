#ifndef Domabot_RosService_h
#define Domabot_RosService_h

#include <domabot_common_lib/Exception.h>

#include <rclcpp/rclcpp.hpp>

#include <stdexcept>

namespace Domabot {

namespace RosService {

  template<typename ServiceType>
  std::shared_ptr<const typename ServiceType::Response> callAndVerifyService(
      const rclcpp::Node::SharedPtr node
    , const rclcpp::Logger& logger
    , const typename rclcpp::Client<ServiceType>::SharedPtr client
    , const std::shared_ptr<typename ServiceType::Request> request
  ) try {
    using namespace std::chrono_literals;
    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        throw Exception::createError("Interrupted while waiting for the service.");
      }
      RCLCPP_INFO(logger, "Service not available, waiting...");
    }

    RCLCPP_INFO(logger, "Send request to service");
    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      throw Exception::createError("Failed to call service!");
    }

    const std::shared_ptr<const typename ServiceType::Response> response = result.get();
    if (!response->response_data.is_success) {
      throw Exception::createError(response->response_data.error_message);
    }

    return response;
  } defaultCatch

} // RosService

} // Domabot

#endif // Domabot_RosService_h
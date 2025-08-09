/**
 * @file RosService.h
 * @brief Domabot RosService tools header file.
 * @details Contains tool method for unified work with ROS services.
 * @copyright Copyright 2025 m79lol
*/
#ifndef DOMABOT_COMMON_LIB__ROSSERVICE_H_
#define DOMABOT_COMMON_LIB__ROSSERVICE_H_

#include <domabot_common_lib/Exception.h>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <stdexcept>
#include <string>

namespace Domabot {

/**
 * @brief Namespace for unified functions to work with ROS all services.
 * @details in Domabot environment.
 */
namespace RosService {

   /**
     * @brief Call to ROS service and verifies its answer by checking common fields.
     * @details Requires use field with type "ResponseData" and name "response_data"
     * in response section within all *.srv files for Domabot environment.
     *
     * @param[in] node Pointer for current ROS node.
     * @param[in] logger Ref to used ROS logger.
     * @param[in] client Pointer to ROS service client (compatible with ServiceType).
     * @param[in] request Pointer to service request of ServiceType.
     * @throws Predefined message if ROS service call fails.
     * @throws If call service success, but service return is_success = false. Then
     * error_message from service will we trowed.
     * @return Pointer to service response.
     */
  template<typename ServiceType>
  std::shared_ptr<const typename ServiceType::Response> callAndVerifyService(
      const rclcpp::Node::SharedPtr node
    , const rclcpp::Logger& logger
    , const typename rclcpp::Client<ServiceType>::SharedPtr client
    , const std::shared_ptr<typename ServiceType::Request> request
  ) try {
    const std::string serviceName = client->get_service_name();

    while (!client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw Exception::createError(
          "Interrupted while waiting for the service: ", serviceName);
      }
      RCLCPP_DEBUG_STREAM(logger,
        "Service \"" << serviceName << "\" not available, waiting...");
    }

    RCLCPP_INFO_STREAM(logger, "Send request to service: " << serviceName);
    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      throw Exception::createError("Failed to call service: ", serviceName);
    }

    const std::shared_ptr<const typename ServiceType::Response> response =
      result.get();
    if (!response->response_data.is_success) {
      throw Exception::createError(response->response_data.error_message);
    }

    RCLCPP_DEBUG_STREAM(logger, "Success response from to service: " << serviceName);

    return response;
  } defaultCatch

}  // namespace RosService

}  // namespace Domabot

#endif  // DOMABOT_COMMON_LIB__ROSSERVICE_H_

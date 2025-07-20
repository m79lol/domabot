#ifndef Domabot_RosService_h
#define Domabot_RosService_h

#include <rtk_common/Exceptions/CException.h>
#include <ros/ros.h>

#include <stdexcept>

namespace Domabot {

namespace RosService {

  template<typename ServiceType>
  void callAndVerifyService(ros::ServiceClient& client, ServiceType& service, bool responseCanBeThrow = true) try {
    if (!client.call(service)) {
      throw makeFormatError("Fail request to ", ros::service_traits::DataType<ServiceType>::value(), "!");
    }
    if (service.response.success) {
      return;
    }
    if (!responseCanBeThrow) {
      ROS_WARN_STREAM("Service response error message: " << service.response.error_message);
      return;
    }
    throw std::runtime_error(service.response.error_message);
  } catchBtRethrow;

} // RosService

} // Domabot

#endif // Domabot_RosService_h
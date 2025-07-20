
#include <domabot_controller/ControllerParams.h>
#include <domabot_controller/RosParam.h>

namespace Domabot {

std::string ControllerParams::checkIsEmpty(const rclcpp::Parameter& parameter) try {
  const std::string str = parameter.as_string();
  if (str.empty()) {
    throw Exception::createError("The parameter is empty string!");
  }
  return str;
} defaultCatch

unsigned int ControllerParams::getBaudRate(const rclcpp::Node::SharedPtr node) try {
  return RosParam<int>(
      node
    , "baud_rate"
    , 9600
    , [](const rclcpp::Parameter& parameter) {
      constexpr const std::array<unsigned int, 10> allowed = {
        2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200, 230400};
      return checkInArray(allowed, (unsigned int)parameter.as_int());
    }
    , "Speed communication with controller over RTU, in bauds. Default is 9600."
  ).getValue();
} defaultCatch

unsigned int ControllerParams::getDataBits(const rclcpp::Node::SharedPtr node) try {
  return RosParam<int>(
      node
    , "data_bits"
    , 8
    , [](const rclcpp::Parameter& parameter) {
      constexpr const std::array<unsigned int, 4> allowed = { 5, 6, 7, 8 };
      return checkInArray(allowed, (unsigned int)parameter.as_int());
    }
    , "Data bits in RTU: [5,6,7,8]. Default is 8."
  ).getValue();
} defaultCatch

char ControllerParams::getParity(const rclcpp::Node::SharedPtr node) try {
  return RosParam<char>(
      node
    , "parity"
    , 'N'
    , [](const rclcpp::Parameter& parameter) {
      constexpr const std::array<char, 3> allowed = { 'N', 'E', 'O' };
      return checkInArray(allowed, checkIsEmpty(parameter).at(0));
    }
    , "Parity for RTU: [N,E,O]. Default is N."
  ).getValue();
} defaultCatch

std::string ControllerParams::getPath(const rclcpp::Node::SharedPtr node) try {
  return RosParam<std::string>(
      node
    , "path"
    , "/dev/ttyUSB0"
    , [](const rclcpp::Parameter& parameter) {
      return checkIsEmpty(parameter);
    }
    , "Path to controller device in system, usually like /dev/ttyUSB0"
  ).getValue();
} defaultCatch

unsigned int ControllerParams::getStopBits(const rclcpp::Node::SharedPtr node) try {
  return RosParam<char>(
      node
    , "stop_bits"
    , '1'
    , [](const rclcpp::Parameter& parameter) {
      constexpr const std::array<unsigned int, 2> allowed = { '1', '2' };
      return checkInArray(allowed, (unsigned int)parameter.as_int());
    }
    , "Stop bits for RTU: [1,2]. Default is 1."
  ).getValue();
} defaultCatch

unsigned int ControllerParams::getSlaveId(const rclcpp::Node::SharedPtr node) try {
  return RosParam<char>(
      node
    , "modbus_slave_id"
    , '1'
    , [](const rclcpp::Parameter& parameter) {
      const int slaveId = parameter.as_int();
      if (slaveId <= 0) {
        throw Exception::createError("Modbus slave id is ", slaveId, ". Slave id must be above zero.");
      }
      return slaveId;
    }
    , "Modbus slave id address. Default is 1."
  ).getValue();
} defaultCatch

} // Domabot
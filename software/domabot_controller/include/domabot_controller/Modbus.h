/**
 * @file Modbus.h
 * @brief Domabot Modbus class header file.
 * @copyright Copyright 2025 m79lol
*/
#ifndef DOMABOT_CONTROLLER__MODBUS_H_
#define DOMABOT_CONTROLLER__MODBUS_H_

#include <domabot_common_lib/Exception.h>

#include <domabot_firmware/firmware_data_types.h>

#include <rclcpp/rclcpp.hpp>

#include <modbus.h>

#include <list>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace Domabot {

/**
 * @brief Modbus communication wrapper class.
 * @details Used libmodbus inside. Requires outside define COIL, REG_INP and REG_HLD
 * enums  of uint8_t type with list of default modbus items: coils, input registers
 * and holding registers addresses. Every enum must have START & END items with
 * start & end address. Blanks between items are prohibited. The input bits not
 * used, but it easy do implement if will needed.
 */
class Modbus {
  public:
    using Ptr = std::shared_ptr<Modbus>;
    using CnstPtr = std::shared_ptr<const Modbus>;

    using Coils            = std::set<COIL>;
    using HoldingRegisters = std::set<REG_HLD>;
    using InputRegisters   = std::set<REG_INP>;

    using CoilsValues            = std::unordered_map<COIL, bool>;
    using HoldingRegistersValues = std::unordered_map<REG_HLD, uint16_t>;
    using InputRegistersValues   = std::unordered_map<REG_INP, uint16_t>;

  protected:
    template <typename T> using WriteFunc = std::function<void(
      const uint8_t, const std::vector<T>&)>;
    template <typename T> using ReadFunc = std::function<std::vector<T>(
      const uint8_t, const size_t)>;

    const rclcpp::Logger m_logger;  ///< Internall ROS logger.
    modbus_t* m_cntx = nullptr;   ///<  Modbus context socket descriptor.
    mutable std::mutex m_mtx;  ///< Mutex for modbus context.
    bool m_isConnected = false; ///< Is device connected flag.

    /**
     * @brief Delay after connect in seconds;
     * @details Needs to establish connection on micro-controller side.
    */
    const double m_connectDelay = 1.0;

    /** @brief Disconnect from modbus, close and free context */
    void free() noexcept;

    /**
     * @brief Run modbus operation wrapper. Execute reconnect on fail.
     * @details Modbus lost connection by timeout during inactivity without any
     * notification. So need reconnection understand only by fails.
     * @param[in] operation Function that performs need operation with Modbus
     * context. Function must return boolean success flag.
     * @throws if operations fails after reconnect.
     */
    void runModbusOperation(
      std::function<bool (modbus_t*)> operation
    );

    /**
     * @brief Checks that item address inside ENUM items bounds.
     * @details Performs validation by START & END enum items.
     * @tparam Item Modbus item type (COIL, REG_INP or REG_HLD).
     * @param[in] items Set of item addressess.
     * @throws If some address outside START & END bounds.
     */
    template <typename Item>
    static void checkInvalidAddresses(const std::set<Item>& items) try {
      std::list<uint8_t> invalids;
      for (const auto& iReg : items) {
        const uint8_t address = (uint8_t) iReg;
        if (address >= (uint8_t) Item::END) {
          invalids.push_back(address);
        }
      }
      if (invalids.empty()) {
        return;
      }

      const auto concat = [](std::string a, const uint8_t b) {
        return std::move(a) + ", " + std::to_string(b);
      };
      const std::string invalidsStr = std::accumulate(
          std::next(invalids.rbegin())
        , invalids.rend()
        , std::to_string(invalids.front())
        , concat);

      throw Exception::createError("Invalid addresses: ", invalidsStr);
    } defaultCatch

    /**
     * @brief Obtain from set of modbus Items continuos addresses sequences.
     * @details Continuos addresses sequences useful to reduce modbus traffic,
     * because allow pack serial address to one request and one package.
     * @tparam Item Modbus item type (COIL, REG_INP or REG_HLD).
     * @param[in] items Set of item addressess.
     * @return Continuos sequences list of addresses modbus items.
     */
    template <typename Item>  // cppcheck-suppress syntaxError
    static std::list<std::list<uint8_t>> getItemsSequences(
      const std::set<Item>& items
    ) try {
      std::list<std::list<uint8_t>> seqs;
      for (auto it = items.begin(); it != items.end(); ) {
        std::list<uint8_t> seq;
        while (it != items.end()) {
          const uint8_t curr = (uint8_t) *it++;
          seq.push_back(curr);
          if (it != items.end()) {
            if (1 != ((uint8_t) *it - curr)) {
              break;
            }
          }
        }
        seqs.push_back(seq);
      }
      return seqs;
    } defaultCatch

    /**
     * @brief Read wrapper for modbus items values from Modbus device.
     * @details Validate item's addresses, compose from it to continuos sequences,
     * and transfer its to read function.
     * @tparam Item Modbus item type (COIL, REG_INP or REG_HLD).
     * @tparam T Type of item value (uint16_t for registers, bool for coils & bits).
     * @param[in] items Set of item addressess.
     * @param[in] readFunc Function that performs read from Modbus context.
     * @return Associative map with item's address & values.
     * @throws If there is invalid item's addresses.
     */
    template <typename Item, typename T> std::unordered_map<Item, T> readItems(
      const std::set<Item>& items,
      const ReadFunc<T>& readFunc
    ) try {
      checkInvalidAddresses<Item>(items);
      const std::list<std::list<uint8_t>> seqs = getItemsSequences<Item>(items);

      std::unordered_map<Item, T> itemValues;
      for (const auto& seq : seqs) {
        const uint8_t startAddress = seq.front();
        const size_t cnt = seq.back() - startAddress + 1;
        const std::vector<T> values = readFunc(startAddress, cnt);

        size_t i = 0;
        for (auto it = seq.begin(); it != seq.end(); ++it) {
          itemValues.emplace((Item) *it, values[i++]);
        }
      }

      return itemValues;
    } defaultCatch

    /**
     * @brief Write wrapper for modbus items values to Modbus device.
     * @details Validate item's addresses, compose from it to continuos sequences,
     * and transfer its to write function.
     * @tparam Item Modbus item type (COIL, REG_INP or REG_HLD).
     * @tparam T Type of item value (uint16_t for registers, bool for coils & bits).
     * @param[in] itemValues Associative map with item's address & values.
     * @param[in] writeFunc Function that performs write to Modbus context.
     * @throws If there is invalid item's addresses.
     */
    template <typename Item, typename T> void writeItems(
      const std::unordered_map<Item, T>& itemValues,
      const WriteFunc<T>& writeFunc
    ) try {
      std::set<Item> items;
      for (const auto& item : itemValues) {
        items.insert(item.first);
      }

      checkInvalidAddresses<Item>(items);
      const std::list<std::list<uint8_t>> seqs = getItemsSequences<Item>(items);

      for (const auto& seq : seqs) {
        const uint8_t startAddress = seq.front();
        const size_t cnt = seq.back() - startAddress + 1;

        size_t i = 0;
        std::vector<T> values(cnt, 0);
        for (auto it = seq.begin(); it != seq.end(); ++it) {
          values[i++] = itemValues.at((Item) *it);
        }
        writeFunc(startAddress, values);
      }
    } defaultCatch

  public:
    /**
     * @brief Single modbus wrapper constructor.
     * @param logger ROS logger for wrapper log output.
     * @param path Path string to serial socket in filesystem.
     * @param baudRate Baud rate speed for serial communication.
     * @param parity Parity of serial communication.
     * @param dataBits Data bits used.
     * @param stopBits Stop bits used.
     * @param slaveId Modbus address of micro-controller in modbus network.
     * @throws On fails create modbus context or assign slave id.
     */
    Modbus(
      const rclcpp::Logger& logger
      , const std::string& path
      , const unsigned int baudRate
      , const char parity
      , const unsigned int dataBits
      , const unsigned int stopBits
      , const unsigned int slaveId
      , const double connectDelay
      , const double modbusTimeout
    );

    Modbus(const Modbus& other)            = delete;
    Modbus(Modbus&& other)                 = delete;
    Modbus& operator=(const Modbus& other) = delete;
    Modbus& operator=(Modbus&& other)      = delete;

    virtual ~Modbus() noexcept;

    /**
     * @brief Read single coil from modbus device.
     * @param address Coil address from COIL enum.
     * @return Coil value.
     * @throws On invalid addresses or something wrong with communication.
     */
    bool readCoil(const COIL address);

    /**
     * @brief Read set of coils from modbus device.
     * @param coils Set of unique coil addresses from COIL enum.
     * @return Associative map coil's addresses & values.
     * @throws On invalid addresses or something wrong with communication.
     */
    CoilsValues readCoils(const Coils& coils);

     /**
     * @brief Write single coil's value by address to modbus device.
     * @param address Coil's address from COIL enum.
     * @param value Coil's value.
     * @throws On invalid addresses or something wrong with communication.
     */
    void writeCoil(const COIL address, const bool value);

    /**
     * @brief Write set of coil's values by addresses to modbus device.
     * @param coilValues Associative map coil's addresses & values.
     * @throws On invalid addresses or something wrong with communication.
     */
    void writeCoils(const CoilsValues& coilValues);

    /**
     * @brief Read single input register from modbus device.
     * @param address Input register address from REG_INP enum.
     * @return Input register value.
     * @throws On invalid addresses or something wrong with communication.
     */
    uint16_t readInputRegister(const REG_INP address);

    /**
     * @brief Read set of input registers from modbus device.
     * @param registers Set of unique input registers addresses from REG_INP enum.
     * @return Associative map input register's addresses & values.
     * @throws On invalid addresses or something wrong with communication.
     */
    InputRegistersValues readInputRegisters(const InputRegisters& registers);

    /**
     * @brief Read single holding register from modbus device.
     * @param address Holding register address from REG_HLD enum.
     * @return Holding register value.
     * @throws On invalid addresses or something wrong with communication.
     */
    uint16_t readHoldingRegister(const REG_HLD address);

    /**
     * @brief Read set of holding registers from modbus device.
     * @param registers Set of unique holding registers addresses from REG_HLD enum.
     * @return Associative map holding register's addresses & values.
     * @throws On invalid addresses or something wrong with communication.
     */
    HoldingRegistersValues readHoldingRegisters(const HoldingRegisters& registers);

     /**
     * @brief Write single holding register's value by address to modbus device.
     * @param address Holding register's address from REG_HLD enum.
     * @param value Holding register's value.
     * @throws On invalid addresses or something wrong with communication.
     */
    void writeHoldingRegister(const REG_HLD address, const uint16_t value);

    /**
     * @brief Write set of holding register's values by addresses to modbus device.
     * @param registerValues Associative map holding register's addresses & values.
     * @throws On invalid addresses or something wrong with communication.
     */
    void writeHoldingRegisters(const HoldingRegistersValues& registerValues);
};  // Modbus

}  // namespace Domabot

#endif  // DOMABOT_CONTROLLER__MODBUS_H_

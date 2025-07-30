#ifndef Domabot_Modbus_h
#define Domabot_Modbus_h

#include <domabot_controller/Exception.h>

#include <domabot_firmware/firmware_data_types.h>

#include <rclcpp/rclcpp.hpp>

#include <modbus.h>

#include <memory>
#include <mutex>
#include <set>
#include <unordered_map>
#include <vector>

namespace Domabot {

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
    template <typename T> using WriteFunc = std::function<void(const uint8_t, const std::vector<T>&)>;
    template <typename T> using ReadFunc = std::function<std::vector<T>(const uint8_t, const size_t)>;

    const rclcpp::Logger m_logger;
    modbus_t* m_cntx = nullptr;
    mutable std::mutex m_mtx;
    bool m_isConnected = false;

    void free() noexcept;

    void runModbusOperation(
      std::function<bool (modbus_t*)> operation
    );

    template <typename Item>
    static std::list<std::list<uint8_t>> getItemsSequences(const std::set<Item>& items) try {
      Exception err("Invalid addresses: ");
      for (const auto& iReg : items) {
        const uint8_t address = (uint8_t) iReg;
        if (address >= (uint8_t) Item::END) {
          err.add(Exception::createMsg("Item with address: ", address));
        }
      }
      err.checkSelf();

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

    template <typename Item, typename T> std::unordered_map<Item, T> readItems(
      const std::set<Item>& items,
      const ReadFunc<T>& readFunc
    ) try {
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

    template <typename Item, typename T> void writeItems(
      const std::unordered_map<Item, T>& itemValues,
      const WriteFunc<T>& writeFunc
    ) try {
      std::set<Item> items;
      for (const auto& item : itemValues) {
        items.insert(item.first);
      }

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
    Modbus(
      const rclcpp::Logger& logger
      , const std::string& path
      , const unsigned int baudRate
      , const char parity
      , const unsigned int dataBits
      , const unsigned int stopBits
      , const unsigned int slaveId
    );

    Modbus(const Modbus& other)            = delete;
    Modbus(Modbus&& other)                 = delete;
    Modbus& operator=(const Modbus& other) = delete;
    Modbus& operator=(Modbus&& other)      = delete;

    virtual ~Modbus() noexcept;

    bool readCoil(const COIL address);

    void writeCoil(const COIL address, const bool value);
    void writeCoils(const CoilsValues& coilValues);

    uint16_t readInputRegister(const REG_INP address);
    InputRegistersValues readInputRegisters(const InputRegisters& registers);

    uint16_t readHoldingRegister(const REG_HLD address);
    HoldingRegistersValues readHoldingRegisters(const HoldingRegisters& registers);
    void writeHoldingRegister(const REG_HLD address, const uint16_t value);
    void writeHoldingRegisters(const HoldingRegistersValues& registerValues);

}; // Modbus

} // Domabot

#endif // Domabot_Modbus_h
#ifndef firmware_data_types_h
#define firmware_data_types_h

#define PROTOCOL_VERSION 1

#define MOTOR_CNT 2

/**
 * @brief Allowed commands in REG_HLD_CMD
 *
 */
enum class CMD : uint8_t {
  BRAKE  = 0,
  STOP   = 1,
  MOVE   = 2,
  UPDATE = 3,
  MODE   = 4,
  DIR    = 5
};

/**
 * @brief Command execution statuses in REG_INP_STS
 *
 */
enum class STS : uint8_t {
  OK          = 0,
  ERR_MOVING  = 1,
  ERR_CMD     = 2,
  ERR_PARAMS  = 3,
  ERR_MODE    = 4,
  ERR_DIR     = 5,
  EMRGENCY    = 6,
  ERR_UNKNOWN = 99
};

/**
 * @brief Modbus Coils addresses
 *
 */
enum class COIL : uint8_t {
  START   = 0,
  NEW_CMD = 0,
  NEW_STS = 1,
  END     = 2
};

/**
 * @brief Modbus Holding Registers addresses
 *
 */
enum class REG_HLD : uint8_t {
  START        = 0,
  CMD          = 0,
  TARG_L       = 1,
  TARG_R       = 2,
  RATE         = 3,
  MAX_SPD_L    = 4,
  MAX_ACC_L    = 5,
  GEAR_L       = 6,
  WHEEL_DIAM_L = 7,
  IS_FROWARD_L = 8,
  MAX_SPD_R    = 9,
  MAX_ACC_R    = 10,
  GEAR_R       = 11,
  WHEEL_DIAM_R = 12,
  IS_FROWARD_R = 13,
  MODE         = 14,
  DIR          = 15,
  END          = 16 // 1+2+1+5*2+2
};

/**
 * @brief Modbus Input Registers addresses
 *
 */
enum class REG_INP : uint8_t {
  START  = 0,
  VER    = 0,
  STS    = 1,
  STPR_L = 2,
  POS_L  = 3,
  STPR_R = 4,
  POS_R  = 5,
  END    = 6
};

/**
 * @brief Operation modes
 *
 */
enum class MODE : uint8_t {
  TRG  = 0,
  DRCT = 1,
  WRD  = 2
};

/**
 * @brief Directions in MANUAL mode
 *
 */
enum class DIR : uint8_t {
  STOP     = 0,
  FORWARD  = 1,
  RIGHT    = 2, // clock wise
  BACKWARD = 3,
  LEFT     = 4 // counter clock wise
};

/**
 * @brief Stepper motor data stored in EEPROM memory
 *
 */
struct StepperData {
  uint16_t maxSpeedMms = 110;
  uint16_t maxAccMms2 = 55;
  uint16_t gearRatio = uint16_t(85.0 / 24.0 * 1000.0); // Z-driven / Z-leading * precision // 3541
  uint8_t wheelDiamMm = 200;
  uint8_t isForward = 1; // must zero for L motor, index 0
};

/**
 * @brief Controller data stored in EEPROM memory
 *
 */
struct ControllerData {
  StepperData stepperData[MOTOR_CNT]; // left is 0, right is 1
  uint16_t updateRateHz = 50; // for Modbus Input Registers
};

#endif // firmware_data_types_h
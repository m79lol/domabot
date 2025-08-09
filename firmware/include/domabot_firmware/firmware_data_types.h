/**
 * @file firmware_data_types.h
 * @brief Header for firmware base enums.
 * @details Contains base enums used in modbus communication between
 * microcontrolher and PC.
 * @copyright Copyright 2025 m79lol
*/
#ifndef DOMABOT_FIRMWARE__FIRMWARE_DATA_TYPES_H_
#define DOMABOT_FIRMWARE__FIRMWARE_DATA_TYPES_H_

#define PROTOCOL_VERSION 1  ///< update with every change in enums below

/** @brief Allowed commands in REG_HLD::CMD. */
enum class CMD : uint8_t {
  BRAKE  = 0,  ///< Emergency immediate stop
  STOP   = 1,  ///< Controlled slow down and stop
  MOVE   = 2,  ///< Move steppers to target
  UPDATE = 3,  ///< Apply controller settings from modbus registers values
  SAVE   = 4,  ///< Performs UPDATE, and them save settings to controller memory
  MODE   = 5,  ///< Change operation mode, see enum MODE
  DIR    = 6   ///< Change moving direction in Direction Mode, see enum DIR
};

/** @brief Command execution statuses in REG_INP::STS. */
enum class STS : uint8_t {
  OK          = 0,  ///< All is good, command completed
  ERR_MOVING  = 1,  ///< Can't execute command during moving
  ERR_CMD     = 2,  ///< Invalid command
  ERR_PARAMS  = 3,  ///< Invalid settings values
  ERR_MODE    = 4,  ///< Invalid requested mode, see enum MODE
  ERR_DIR     = 5,  ///< Invalid requested direction, see enum DIR
  EMERGENCY   = 6,  ///< Emergency is active (by hardware)
  ERR_UNKNOWN = 99  ///< Unknown error
};

/** @brief Modbus Coils addresses. */
enum class COIL : uint8_t {
  START   = 0,  ///< Start address (only for internal use)
  NEW_CMD = 0,  ///< 1 command sended (set by PC), 0 is command received & accepted (set by micro)
  NEW_STS = 1,  ///< 1 status updated (set by micro), 0 is status received (set by PC)
  END     = 2   ///< End address (only for internal use)
};

/** @brief Modbus Holding Registers addresses. */
enum class REG_HLD : uint8_t {
  START        = 0,   ///< Start address (only for internal use)
  CMD          = 0,   ///< Command register
  TARG_L       = 1,   ///< Target for left stepper, mm
  TARG_R       = 2,   ///< Target for right stepper, mm
  RATE         = 3,   ///< Update status rate, hz
  MAX_SPD_L    = 4,   ///< Max speed for left stepper, mm/sec
  MAX_ACC_L    = 5,   ///< Max acceleration for left stepper, mm/sec
  GEAR_L       = 6,   ///< Gear ration for left stepper
  WHEEL_DIAM_L = 7,   ///< Wheel diameter for left stepper, mm
  IS_FROWARD_L = 8,   ///< Rotation direction for left stepper
  MAX_SPD_R    = 9,   ///< Max speed for right stepper, mm/sec
  MAX_ACC_R    = 10,  ///< Max acceleration for right stepper, mm/sec
  GEAR_R       = 11,  ///< Gear ration for right stepper
  WHEEL_DIAM_R = 12,  ///< Wheel diameter for right stepper, mm
  IS_FROWARD_R = 13,  ///< Rotation direction for right stepper
  MODE         = 14,  ///< Requested operation mode for MODE command, see MODE enum
  DIR          = 15,  ///< Requested direction for DIR command, see DIR enum
  END          = 16   ///< End address (only for internal use)
};

/** @brief Modbus Input Registers addresses. */
enum class REG_INP : uint8_t {
  START  = 0,  ///< Start address (only for internal use)
  VER    = 0,  ///< Used communication protocol version, see PROTOCOL_VERSION
  STS    = 1,  ///< Command post-execution status, see enum STS
  STPR_L = 2,  ///< Current left stepper status, see enum STPR_STS
  POS_L  = 3,  ///< Current left stepper position, mm
  STPR_R = 4,  ///< Current right stepper status, see enum STPR_STS
  POS_R  = 5,  ///< Current right stepper position, mm
  END    = 6   ///< End address (only for internal use)
};

/** @brief Operation modes. */
enum class MODE : uint8_t {
  TRG  = 0,  ///< Move to target positions by TARG_L & TARG_R holding registers
  DRCT = 1,  ///< Move in target direction by DIR holding register
  WRD  = 2   ///< Move by wired remote control (this mode activate only by hardware switch)
};

/** @brief Directions in MANUAL mode. */
enum class DIR : uint8_t {
  STOP     = 0,  ///< Stop movement, performs STOP command
  FORWARD  = 1,  ///< Move in forward direction
  RIGHT    = 2,  ///< Rotation on RIGHT (in clock wise)
  BACKWARD = 3,  ///< Move in backward direction
  LEFT     = 4   ///< Rotation on LEFT (in counter clock wise)
};

/** @brief Stepper statuses from GyverStepper lib. */
enum class STPR_STS : uint8_t {
  STOPPED                = 0,  ///< No motion
  MOVING_TO_TARGET       = 1,  ///< Moving to target (used in TRG mode)
  MOVING_TO_PAUSE_POINT  = 2,  ///< Never used
  MOVING_AT_SPEED        = 3,  ///< Moving at required speed (used in DRCT & WRD modes)
  SLOWING_DOWN           = 4   ///< Appears during STOP command execution
};

#endif  // DOMABOT_FIRMWARE__FIRMWARE_DATA_TYPES_H_

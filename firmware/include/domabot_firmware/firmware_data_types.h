/*!
\file
\brief Header for firmware base enums

Contains base enums used in modbus communication between microcontrolher and PC.
*/

#ifndef firmware_data_types_h
#define firmware_data_types_h

#define PROTOCOL_VERSION 1 ///< update with every change in enums below

/**
 * @brief Allowed commands in REG_HLD::CMD
 *
 */
enum class CMD : uint8_t {
  BRAKE  = 0, ///< emergency immediate stop
  STOP   = 1, ///< controlled slow down and stop
  MOVE   = 2, ///< move steppers to target
  UPDATE = 3, ///< apply controller settings from modbus registers values
  SAVE   = 4, ///< performs UPDATE, and them save settings to controller memory
  MODE   = 5, ///< change operation mode, see enum MODE
  DIR    = 6  ///< change moving direction in Direction Mode, see enum DIR
};

/**
 * @brief Command execution statuses in REG_INP::STS
 *
 */
enum class STS : uint8_t {
  OK          = 0, ///< all is good, command completed
  ERR_MOVING  = 1, ///< can't execute command during moving
  ERR_CMD     = 2, ///< invalid command
  ERR_PARAMS  = 3, ///< invalid settings values
  ERR_MODE    = 4, ///< invalid requested mode, see enum MODE
  ERR_DIR     = 5, ///< invalid requested direction, see enum DIR
  EMERGENCY   = 6, ///< emergency is active (by hardware)
  ERR_UNKNOWN = 99 ///< unknown error
};

/**
 * @brief Modbus Coils addresses
 *
 */
enum class COIL : uint8_t {
  START   = 0, ///< start address (only for internal use)
  NEW_CMD = 0, ///< 1 command sended (set by PC), 0 is command received & accepted (set by micro)
  NEW_STS = 1, ///< 1 status updated (set by micro), 0 is status received (set by PC)
  END     = 2  ///< end address (only for internal use)
};

/**
 * @brief Modbus Holding Registers addresses
 *
 */
enum class REG_HLD : uint8_t {
  START        = 0,  ///< start address (only for internal use)
  CMD          = 0,  ///< command register
  TARG_L       = 1,  ///< target for left stepper, mm
  TARG_R       = 2,  ///< target for right stepper, mm
  RATE         = 3,  ///< update status rate, hz
  MAX_SPD_L    = 4,  ///< max speed for left stepper, mm/sec
  MAX_ACC_L    = 5,  ///< max acceleration for left stepper, mm/sec
  GEAR_L       = 6,  ///< gear ration for left stepper
  WHEEL_DIAM_L = 7,  ///< wheel diameter for left stepper, mm
  IS_FROWARD_L = 8,  ///< rotation direction for left stepper
  MAX_SPD_R    = 9,  ///< max speed for right stepper, mm/sec
  MAX_ACC_R    = 10, ///< max acceleration for right stepper, mm/sec
  GEAR_R       = 11, ///< gear ration for right stepper
  WHEEL_DIAM_R = 12, ///< wheel diameter for right stepper, mm
  IS_FROWARD_R = 13, ///< rotation direction for right stepper
  MODE         = 14, ///< requested operation mode for MODE command, see MODE enum
  DIR          = 15, ///< requested direction for DIR command, see DIR enum
  END          = 16  ///< end address (only for internal use)
};

/**
 * @brief Modbus Input Registers addresses
 *
 */
enum class REG_INP : uint8_t {
  START  = 0, ///< start address (only for internal use)
  VER    = 0, ///< used communication protocol version, see PROTOCOL_VERSION
  STS    = 1, ///< command post-execution status, see enum STS
  STPR_L = 2, ///< current left stepper status, see enum STPR_STS
  POS_L  = 3, ///< current left stepper position, mm
  STPR_R = 4, ///< current right stepper status, see enum STPR_STS
  POS_R  = 5, ///< current right stepper position, mm
  END    = 6  ///< end address (only for internal use)
};

/**
 * @brief Operation modes
 *
 */
enum class MODE : uint8_t {
  TRG  = 0, ///< move to target positions by TARG_L & TARG_R holding registers
  DRCT = 1, ///< move in target direction by DIR holding register
  WRD  = 2  ///< move by wired remote control (this mode activate only by hardware switch)
};

/**
 * @brief Directions in MANUAL mode
 *
 */
enum class DIR : uint8_t {
  STOP     = 0, ///< stop movement, performs STOP command
  FORWARD  = 1, ///< move in forward direction
  RIGHT    = 2, ///< rotation on RIGHT (in clock wise)
  BACKWARD = 3, ///< move in backward direction
  LEFT     = 4  ///< rotation on LEFT (in counter clock wise)
};

/**
 * @brief Stepper statuses
 *
 * from GyverStepper lib
 */
enum class STPR_STS : uint8_t {
  STOPPED                = 0, ///< no motion
  MOVING_TO_TARGET       = 1, ///< moving to target (used in TRG mode)
  MOVING_TO_PAUSE_POINT  = 2, ///< never used
  MOVING_AT_SPEED        = 3, ///< moving at required speed (used in DRCT & WRD modes)
  SLOWING_DOWN           = 4  ///< appears during STOP command execution
};

#endif // firmware_data_types_h
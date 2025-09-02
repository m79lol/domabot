/**
 * @file main.ino
 * @brief Main firmware source file.
 * @details Contains firmware C++ code for microcontrolher based Arduino Mega2560.
 * @copyright Copyright 2025 m79lol
*/

#include "domabot_firmware/firmware_data_types.h"

#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

#include <EEManager.h>

#include <GyverStepper2.h>

#include <math.h>

#define FIRMWARE_VERSION 1  ///< Update with every firmware release

#define MODBUS_SLAVE_ID 1  ///< Device addres in modbus network
#define SERIAL_BAUD_RATE 9600  ///< Default: 9600.
// Allowed: 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200, 23040

// PINS assignation
#define MOTOR_L_STEP_PIN 11
#define MOTOR_L_DIR_PIN  12
#define MOTOR_L_EN_PIN   13

#define MOTOR_R_STEP_PIN 9
#define MOTOR_R_DIR_PIN  8
#define MOTOR_R_EN_PIN   10

#define WRD_MODE_SWITCH_PIN 9999
#define DIR_FORWARD_PIN     9999
#define DIR_RIGHT_PIN       9999
#define DIR_BACKWARD_PIN    9999
#define DIR_LEFT_PIN        9999

#define EMERGENCY_STOP_PIN  9999
// end PINS

#define STEPS_REV 1600  ///< Steps per motor revolution
// Must be above zero. Sets on stepper driver.
static_assert(0 < STEPS_REV);

#define MOTOR_CNT 2  ///< Motor count is two, always.

/** @brief Stepper motor data stored in EEPROM memory. */
struct StepperData {
  uint16_t maxSpeedMms = 110;
  uint16_t maxAccMms2 = 55;
  uint16_t gearRatio =
    uint16_t(85.0 / 24.0 * 1000.0);  ///< Z-driven / Z-leading * precision = 3541
  uint8_t wheelDiamMm = 200;
  uint8_t isForward = 1;  ///< must zero for L motor, index 0
};

/** @brief Controller data stored in EEPROM memory. */
struct ControllerData {
  StepperData stepperData[MOTOR_CNT];  ///< left is 0, right is 1
  uint16_t updateRateHz = 1;  ///< for Modbus Input Registers
};

uint8_t isSteppersMoving = 0;
bool isMotorsEnabled = false;
MODE mode = MODE::TRG;
DIR direction = DIR::STOP;
const ControllerData defaultControllerData;
ControllerData controllerData;
EEManager memory(controllerData);

GStepper2<STEPPER2WIRE> steppers[MOTOR_CNT] = {
  GStepper2<STEPPER2WIRE>(
    STEPS_REV,
    MOTOR_L_STEP_PIN,
    MOTOR_L_DIR_PIN,
    MOTOR_L_EN_PIN
  ),
  GStepper2<STEPPER2WIRE>(
    STEPS_REV,
    MOTOR_R_STEP_PIN,
    MOTOR_R_DIR_PIN,
    MOTOR_R_EN_PIN
  )
};

/**
 * @brief Modbus holding register read wrapper.
 * @param[in] address Address of holding register.
 * @return Register value.
 */
int32_t holdingRegisterRead(const REG_HLD address) {
  return ModbusRTUServer.holdingRegisterRead(static_cast<int>(address));
}

/**
 * @brief Modbus holding register write wrapper.
 * @param[in] address Address of holding register.
 * @param[in] value Value to write.
 * @return 1 for success, 0 for fail.
 */
int holdingRegisterWrite(const REG_HLD address, const uint16_t value) {
  return ModbusRTUServer.holdingRegisterWrite(static_cast<int>(address), value);
}

/**
 * @brief Modbus input register write wrapper.
 * @param[in] address Address of input register.
 * @param[in] value Value to write.
 * @return 1 for success, 0 for fail.
 */
int inputRegisterWrite(const REG_INP address, const uint16_t value) {
  return ModbusRTUServer.inputRegisterWrite(static_cast<int>(address), value);
}

/**
 * @brief Modbus coil write wrapper.
 * @param[in] address Address of coil.
 * @param[in] value Value to write.
 * @return 1 for success, 0 for fail.
 */
int coilWrite(const COIL address, const uint8_t value) {
  return ModbusRTUServer.coilWrite(static_cast<int>(address), value);
}

/**
 * @brief Modbus coil read wrapper.
 * @param[in] address Address of coil.
 * @return Read value.
 */
bool coilRead(const COIL address) {
  return 0 != ModbusRTUServer.coilRead(static_cast<int>(address));
}

/**
 * @brief Convert distance in mm to motor steps.
 * @param[in] stepperData actual stepper.
 * @param[in] distance in mm.
 * @return Steps count.
 */
int32_t mmToSteps(const StepperData& stepperData, const int16_t distance) {
  if (0 == stepperData.wheelDiamMm) {
    return 0;
  }
  return static_cast<int32_t>(
     static_cast<double>(distance) /
    (static_cast<double>(stepperData.wheelDiamMm) * M_PI) *
    (static_cast<double>(stepperData.gearRatio) / 1000.0) *
     static_cast<double>(STEPS_REV)
  );
}

/**
 * @brief Convert motor steps to distance in mm.
 * @param[in] stepperData Actual stepper.
 * @param[in] steps Count steps.
 * @return Distance.
 */
int16_t stepsToMm(const StepperData& stepperData, const int32_t steps) {
  if (0 == STEPS_REV || 0 == stepperData.gearRatio) {
    return 0;
  }
  return static_cast<int16_t>(
    (static_cast<double>(steps) / static_cast<double>(STEPS_REV)) /
    (static_cast<double>(stepperData.gearRatio) / 1000.0) *
    (static_cast<double>(stepperData.wheelDiamMm) * M_PI)
  );
}

/** @brief Re-Init steppers by actual stepper data. */
void initSteppers() {
  static bool isSteppersInited = false;
  if (isSteppersInited) {
    if (isMotorsEnabled) {
      for (uint8_t i = 0; i < MOTOR_CNT; ++i) {
        steppers[i].brake();
        steppers[i].disable();
      }
    }
    isSteppersInited = false;
  }

  for (uint8_t i = 0; i < MOTOR_CNT; ++i) {
    const StepperData& stepperData = controllerData.stepperData[i];
    steppers[i].setMaxSpeed(mmToSteps(stepperData, stepperData.maxSpeedMms));
    steppers[i].setAcceleration(uint16_t(mmToSteps(
      stepperData, stepperData.maxAccMms2)));
    steppers[i].reverse(0 == stepperData.isForward);
    if (isMotorsEnabled) {
      steppers[i].enable();
    }
  }
  isSteppersInited = true;

  holdingRegisterWrite(REG_HLD::CMD,  (uint16_t) CMD::BRAKE);
  holdingRegisterWrite(REG_HLD::MODE, (uint16_t) mode);
  holdingRegisterWrite(REG_HLD::DIR,  (uint16_t) direction);
}

/**
 * @brief Update Modbus data by finished command.
 * @param[in] status STS status to output in Modbus status register.
 */
void updateStatus(const STS status) {
  inputRegisterWrite(REG_INP::STS, (uint16_t) status);
  coilWrite(COIL::NEW_STS, 1);
}

/**
 * @brief Complete Modbus command execution by full stop & status output.
 * @param[in] status STS status of command execution.
 */
void completeCommand(const STS status) {
  holdingRegisterWrite(REG_HLD::CMD, (uint16_t) CMD::BRAKE);
  updateStatus(status);
}

/**
 * @brief Load controller data from EEPROM memory.
 * @return Exit code.
 */
uint8_t loadFromMemory() {
  const uint8_t res = memory.begin(0, FIRMWARE_VERSION);

  if (0 == controllerData.updateRateHz) {
    controllerData.updateRateHz = defaultControllerData.updateRateHz;
  }
  holdingRegisterWrite(REG_HLD::RATE, controllerData.updateRateHz);

  for (uint8_t i = 0; i < MOTOR_CNT; ++i) {
    StepperData& stepperData = controllerData.stepperData[i];
    const StepperData& defaultData = defaultControllerData.stepperData[i];

    if (0 == stepperData.gearRatio) {
      stepperData.gearRatio = defaultData.gearRatio;
    }
    if (0 == stepperData.wheelDiamMm) {
      stepperData.wheelDiamMm = defaultData.wheelDiamMm;
    }

    const uint8_t baseIndex =
      (uint8_t)(0 == i ? REG_HLD::MAX_SPD_L : REG_HLD::MAX_SPD_R);
    holdingRegisterWrite((REG_HLD)(baseIndex + 0), stepperData.maxSpeedMms);  // 4 9
    holdingRegisterWrite((REG_HLD)(baseIndex + 1), stepperData.maxAccMms2 );  // 5 10
    holdingRegisterWrite((REG_HLD)(baseIndex + 2), stepperData.gearRatio  );  // 6 11
    holdingRegisterWrite((REG_HLD)(baseIndex + 3), stepperData.wheelDiamMm);  // 7 12
    holdingRegisterWrite((REG_HLD)(baseIndex + 4), stepperData.isForward  );  // 8 13
  }

  return res;
}

/** @brief Detect command by Modbus & execute it. */
void processDirection() {
  for (uint8_t i = 0; i < MOTOR_CNT; ++i) {
    const StepperData& stepperData = controllerData.stepperData[i];
    switch (direction) {
      case DIR::STOP: {
        steppers[i].stop();
        break;
      }
      case DIR::FORWARD: {
        steppers[i].setSpeed(mmToSteps(stepperData, stepperData.maxSpeedMms));
        break;
      }
      case DIR::RIGHT: {
        steppers[i].setSpeed(
          mmToSteps(stepperData, stepperData.maxSpeedMms)
          * (i % 2 ? -1 : 1)
        );
        break;
      }
      case DIR::BACKWARD: {
        steppers[i].setSpeed(-mmToSteps(stepperData, stepperData.maxSpeedMms));
        break;
      }
      case DIR::LEFT: {
        steppers[i].setSpeed(
          mmToSteps(stepperData, stepperData.maxSpeedMms)
          * (i % 2 ? 1 : -1)
        );
        break;
      }
      default: {}
    }
  }
}

/** @brief Detect command by Modbus & execute it. */
void processCommand() {
  const int packetReceived = ModbusRTUServer.poll();
  if (!packetReceived) {
    return;  // no new packets
  }
  const bool isCommandSend = coilRead(COIL::NEW_CMD);
  if (!isCommandSend) {
    return;  // no new commands
  }
  // new command received
  coilWrite(COIL::NEW_CMD, 0);
  const CMD command = (CMD) holdingRegisterRead(REG_HLD::CMD);

  switch (command) {
    case CMD::BRAKE:
    case CMD::DIR:
    case CMD::MOVE:
    case CMD::STOP: {
      if (!isMotorsEnabled) {
        completeCommand(STS::ERR_ENBL);
        return;
      }
    }
    default: { break; }
  }

  switch (command) {  // priority commands
    case CMD::BRAKE:
    case CMD::STOP:
    case CMD::MODE: {
      for (uint8_t i = 0; i < MOTOR_CNT; ++i) {
        switch (command) {
          case CMD::BRAKE:  // fall down
          case CMD::MODE: { steppers[i].brake(); break; }
          case CMD::STOP: { steppers[i].stop();  break; }
          default: {}  // warning supress
        }
      }

      if (CMD::MODE != command) {
        completeCommand(STS::OK);
        return;
      }

      direction = DIR::STOP;
      mode = (MODE) holdingRegisterRead(REG_HLD::MODE);
      STS status = STS::OK;
      switch (mode) {
        case MODE::TRG:
        case MODE::DRCT: { break; }
        // case MODE::WRD: only by hardware switch
        default: {
          mode = MODE::TRG;
          status = STS::ERR_MODE;
          holdingRegisterWrite(REG_HLD::MODE, (uint16_t) mode);
          break;
        }
      }
      holdingRegisterWrite(REG_HLD::DIR, (uint16_t) DIR::STOP);
      completeCommand(status);
      return;
    }
    case CMD::DIR: {
      if (MODE::DRCT != mode) {
        completeCommand(STS::ERR_MODE);
        return;
      }
      direction = (DIR) holdingRegisterRead(REG_HLD::DIR);
      STS status = STS::OK;
      switch (direction) {
        case DIR::STOP:
        case DIR::FORWARD:
        case DIR::RIGHT:
        case DIR::BACKWARD:
        case DIR::LEFT: { break; }
        default: {
          direction = DIR::STOP;
          status = STS::ERR_DIR;
          holdingRegisterWrite(REG_HLD::DIR, (uint16_t) DIR::STOP);
          break;
        }
      }

      processDirection();
      updateStatus(status);
      return;
    }
    default: {}  // warning supress
  }

  if (isSteppersMoving) {
    // moving now, ignore any others commands
    completeCommand(STS::ERR_MOVING);
    return;
  }

  // stopped now, process other commands
  switch (command) {
    case CMD::MOVE: {
      if (MODE::TRG != mode) {
        completeCommand(STS::ERR_MODE);
        break;
      }

      for (uint8_t i = 0; i < MOTOR_CNT; ++i) {
        const REG_HLD reg = 0 == i ? REG_HLD::TARG_L : REG_HLD::TARG_R;
        const int16_t target = holdingRegisterRead(reg);
        const StepperData& stepperData = controllerData.stepperData[i];
        steppers[i].setTarget(mmToSteps(stepperData, target));
      }
      completeCommand(STS::OK);
      break;
    };
    case CMD::UPDATE:
    case CMD::SAVE: {
      STS status = STS::OK;
      controllerData.updateRateHz = holdingRegisterRead(REG_HLD::RATE);
      if (0 == controllerData.updateRateHz) {
        status = STS::ERR_PARAMS;
      }

      for (uint8_t i = 0; i < MOTOR_CNT; ++i) {
        StepperData& stepperData = controllerData.stepperData[i];
        const uint8_t baseIndex =
          (uint8_t)(0 == i ? REG_HLD::MAX_SPD_L : REG_HLD::MAX_SPD_R);
        stepperData.maxSpeedMms =
          holdingRegisterRead((REG_HLD)(baseIndex + 0));  // 4 9
        stepperData.maxAccMms2  =
          holdingRegisterRead((REG_HLD)(baseIndex + 1));  // 5 10
        stepperData.gearRatio   =
          holdingRegisterRead((REG_HLD)(baseIndex + 2));  // 6 11
        stepperData.wheelDiamMm =
          holdingRegisterRead((REG_HLD)(baseIndex + 3));  // 7 12
        stepperData.isForward   =
          holdingRegisterRead((REG_HLD)(baseIndex + 4));  // 8 13

        if (0 == stepperData.maxSpeedMms) {
          status = STS::ERR_PARAMS;
        }
        if (0 == stepperData.maxAccMms2) {
          status = STS::ERR_PARAMS;
        }
        if (0 == stepperData.gearRatio) {
          status = STS::ERR_PARAMS;
        }
        if (0 == stepperData.wheelDiamMm) {
          status = STS::ERR_PARAMS;
        }
      }

      if (STS::OK == status && CMD::SAVE == command) {
        memory.updateNow();
        loadFromMemory();
      }
      initSteppers();

      completeCommand(status);
      break;
    };
    case CMD::ENBL: {
      isMotorsEnabled = coilRead(COIL::ENBL);
       for (uint8_t i = 0; i < MOTOR_CNT; ++i) {
        if (isMotorsEnabled) {
          steppers[i].enable();
        } else {
          steppers[i].disable();
        }
      }
      completeCommand(STS::OK);
      break;
    }
    default: {
      completeCommand(STS::ERR_CMD);
      break;
    }
  }
}

/**
 * @brief Execute emergency stop motors.
 * @return true if emergency pin raised.
 */
bool processEmergency() {
  static bool isWasEmergency = false;
  const bool isEmergency = digitalRead((uint8_t) EMERGENCY_STOP_PIN);
  if (isEmergency && !isWasEmergency) {
    for (uint8_t i = 0; i < MOTOR_CNT; ++i) {
      steppers[i].stop();
    }
    isWasEmergency = true;

    // drop active command
    coilWrite(COIL::NEW_CMD, 0);
    completeCommand(STS::EMERGENCY);
  } else if (!isEmergency && isWasEmergency) {
    isWasEmergency = false;
    updateStatus(STS::OK);
  }
  return isEmergency;
}

/** @brief Process all signals from wired remote control. */
void processWires() {
  static MODE storedMode = mode;
  if (digitalRead((uint8_t) WRD_MODE_SWITCH_PIN)) {
    if (MODE::WRD != mode) {
      for (uint8_t i = 0; i < MOTOR_CNT; ++i) {
        steppers[i].brake();
      }
      storedMode = mode;
      mode = MODE::WRD;
    }

    if (digitalRead((uint8_t) DIR_FORWARD_PIN)) {
      direction = DIR::FORWARD;
    } else if (digitalRead((uint8_t) DIR_RIGHT_PIN)) {
      direction = DIR::RIGHT;
    } else if (digitalRead((uint8_t) DIR_BACKWARD_PIN)) {
      direction = DIR::BACKWARD;
    } else if (digitalRead((uint8_t) DIR_LEFT_PIN)) {
      direction = DIR::LEFT;
    } else {
      direction = DIR::STOP;
    }
  } else {
    if (MODE::WRD == mode) {
      for (uint8_t i = 0; i < MOTOR_CNT; ++i) {
        steppers[i].brake();
      }
      mode = storedMode;
    }
  }
}

/** @brief Update steppers data in modbus registers. */
void updateCurrentStatus() {
  static unsigned long future = 0;
  const unsigned long now = millis();
  if (now < future) {
    return;
  }

  future = millis() + 1000 / controllerData.updateRateHz;

  // update current status
  isSteppersMoving = 0;
  for (uint8_t i = 0; i < MOTOR_CNT; ++i) {
    const uint8_t baseIndex = (uint8_t)(0 == i ? REG_INP::STPR_L : REG_INP::STPR_R);
    const uint8_t stepperStatus =  steppers[i].getStatus();
    isSteppersMoving |= stepperStatus;
    inputRegisterWrite((REG_INP)(baseIndex + 0), stepperStatus);

    const int16_t currentMm = stepsToMm(
      controllerData.stepperData[i], steppers[i].getCurrent());
    inputRegisterWrite((REG_INP)(baseIndex + 1), currentMm);
  }
}

/**
 * @brief Check critical condition and stops next execution if it's true.
 * @param[in] check Condition.
 * @param[in] msg Message for print to Serial port.
 */
void checkCriticalError(const bool check, const char* msg) {
  if (!check) {
    Serial.println(msg);
    while (1) {}
  }
}

/** @brief Main setup procedure, execute once by start. */
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  controllerData.stepperData[0].isForward = 0;

  checkCriticalError(
    ModbusRTUServer.begin(MODBUS_SLAVE_ID, SERIAL_BAUD_RATE),
    "Failed to start Modbus RTU Server!"
  );
  checkCriticalError(
      ModbusRTUServer.configureCoils(
        static_cast<int>(COIL::START), static_cast<int>(COIL::END))
    , "Failed to configure Coils"
  );
  checkCriticalError(
      ModbusRTUServer.configureInputRegisters(
        static_cast<int>(REG_INP::START), static_cast<int>(REG_INP::END))
    , "Failed to configure Input Registers"
  );
  checkCriticalError(
      ModbusRTUServer.configureHoldingRegisters(
        static_cast<int>(REG_HLD::START), static_cast<int>(REG_HLD::END))
    , "Failed to configure Holding Registers"
  );

  switch (loadFromMemory()) {
    case 0:  { break; }  // ok, loaded from memory
    case 1:  { break; }  // wrote default
    case 2:  { Serial.println("Memory: too large size"); while (1); }
    default: { Serial.println("Memory: unknown error");  while (1); }
  }

  mode = MODE::TRG;
  direction = DIR::STOP;

  inputRegisterWrite(REG_INP::VER, PROTOCOL_VERSION);

  initSteppers();
  updateStatus(STS::OK);

  // init wires
  pinMode((uint8_t) WRD_MODE_SWITCH_PIN, INPUT);
  pinMode((uint8_t) DIR_FORWARD_PIN,     INPUT);
  pinMode((uint8_t) DIR_RIGHT_PIN,       INPUT);
  pinMode((uint8_t) DIR_BACKWARD_PIN,    INPUT);
  pinMode((uint8_t) DIR_LEFT_PIN,        INPUT);

  pinMode((uint8_t) EMERGENCY_STOP_PIN,  INPUT);
}

/** @brief Main loop of controller, infinite. */
void loop() {
  for (uint8_t i = 0; i < MOTOR_CNT; ++i) {
    steppers[i].tick();  // process motors
  }

  updateCurrentStatus();
  if (processEmergency()) {
    return;  // there is emergency, stop processing
  }

  processWires();

  switch (mode) {
    case MODE::TRG:
    case MODE::DRCT: {
      processCommand();
      break;
    }
    case MODE::WRD: {
      processDirection();
      break;
    }
    default: { break; }
  }
}

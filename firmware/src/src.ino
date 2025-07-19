#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

#include <EEManager.h>

#include <GyverStepper2.h>

#include <math.h>

#define FIRMWARE_VERSION 1
#define PROTOCOL_VERSION 1

#define SERIAL_BAUD_RATE 9600

#define MOTOR_CNT 2
#define STEPS_REV 1600 // must be above zero

static_assert(0 < STEPS_REV);

#define MOTOR_L_STEP_PIN 11
#define MOTOR_L_DIR_PIN  12
#define MOTOR_L_EN_PIN   13

#define MOTOR_R_STEP_PIN 9
#define MOTOR_R_DIR_PIN  8
#define MOTOR_R_EN_PIN   10

#define WRD_MODE_SWITCH_PIN 9999999
#define DIR_FORWARD_PIN     9999999
#define DIR_RIGHT_PIN       9999999
#define DIR_BACKWARD_PIN    9999999
#define DIR_LEFT_PIN        9999999

#define EMERGENCY_STOP_PIN  9999999

/**
 * @brief Allowed commands in REG_HLD_CMD
 *
 */
enum CMDS {
  CMD_BRAKE  = 0,
  CMD_STOP   = 1,
  CMD_MOVE   = 2,
  CMD_UPDATE = 3,
  CMD_MODE   = 4,
  CMD_DIR    = 5
};

/**
 * @brief Command execution statuses in REG_INP_STS
 *
 */
enum STS {
  STS_OK = 0,
  STS_ERR_MOVING  = 1,
  STS_ERR_CMD     = 2,
  STS_ERR_PARAMS  = 3,
  STS_ERR_MODE    = 4,
  STS_ERR_DIR     = 5,
  STS_EMRGENCY    = 6,
  STS_ERR_UNKNOWN = 99
};

/**
 * @brief Modbus Coils addresses
 *
 */
enum COIL_ADDR {
  COIL_START   = 0,
  COIL_NEW_CMD = 0,
  COIL_NEW_STS = 1,
  COIL_END     = 2
};

/**
 * @brief Modbus Holding Registers addresses
 *
 */
enum REG_HLD {
  REG_HLD_START  = 0,
  REG_HLD_CMD    = 0,
  REG_HLD_TARG_L = 1,   // starts from
  REG_HLD_TARG_R = 2,
  REG_HLD_RATE   = 3,
  REG_HLD_SPD_L  = 4, // starts from
  REG_HLD_SPD_R  = 9, // starts from
  REG_HLD_MODE   = 14,
  REG_HLD_DIR    = 15,
  REG_HLD_END    = 16 // 1+2+1+5*2+2
};

/**
 * @brief Modbus Input Registers addresses
 *
 */
enum REG_INP_CTRL {
  REG_INP_START  = 0,
  REG_INP_VER    = 0,
  REG_INP_STS    = 1,
  REG_INP_STPR_L = 2, // starts from
  REG_INP_POS_L  = 3,
  REG_INP_STPR_R = 4,  // starts from
  REG_INP_POS_R  = 5,
  REG_INP_END    = 6
};

/**
 * @brief Operation modes
 *
 */
enum MODE {
  MODE_TRG = 0,
  MODE_DRCT = 1,
  MODE_WRD = 2
};
MODE mode = MODE_TRG;

/**
 * @brief Directions in MANUAL mode
 *
 */
enum DIR {
  DIR_STOP     = 0,
  DIR_FORWARD  = 1,
  DIR_RIGHT    = 2, // clock wise
  DIR_BACKWARD = 3,
  DIR_LEFT     = 4 // counter clock wise
};
DIR direction = DIR_STOP;

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

uint8_t isSteppersMoving = 0;

/**
 * @brief Convert distance in mm to motor steps
 *
 * @param stepperData actual stepper
 * @param distance in mm
 * @return int32_t steps count
 */
int32_t mmToSteps(const StepperData& stepperData, const int16_t distance) {
  if (0 == stepperData.wheelDiamMm) {
    return 0;
  }
  return int32_t(
    double(distance) / (double(stepperData.wheelDiamMm) * M_PI)
      * (double(stepperData.gearRatio)/1000.0) * double(STEPS_REV)
  );
}

/**
 * @brief Convert motor steps to distance in mm
 *
 * @param stepperData actual stepper
 * @param steps count
 * @return int16_t distance
 */
int16_t stepsToMm(const StepperData& stepperData, const int32_t steps) {
  if (0 == STEPS_REV || 0 == stepperData.gearRatio) {
    return 0;
  }
  return int16_t(
    double(steps) / double(STEPS_REV)
    / (double(stepperData.gearRatio)/1000.0)
    * (double(stepperData.wheelDiamMm) * M_PI)
  );
}

/**
 * @brief Re-Init steppers by actual stepper data
 *
 */
void initSteppers() {
  static bool isSteppersInited = false;
  if (isSteppersInited) {
    for (byte i = 0; i < MOTOR_CNT; ++i) {
      steppers[i].brake();
      steppers[i].disable();
    }
    isSteppersInited = false;
  }

  for (byte i = 0; i < MOTOR_CNT; ++i) {
    const StepperData& stepperData = controllerData.stepperData[i];
    steppers[i].setMaxSpeed(mmToSteps(stepperData, stepperData.maxSpeedMms));
    steppers[i].setAcceleration(uint16_t(mmToSteps(stepperData, stepperData.maxAccMms2)));
    steppers[i].reverse(0 == stepperData.isForward);
    steppers[i].enable();
  }
  isSteppersInited = true;

  ModbusRTUServer.holdingRegisterWrite(REG_HLD_CMD, CMD_BRAKE);
  ModbusRTUServer.holdingRegisterWrite(REG_HLD_MODE, mode);
  ModbusRTUServer.holdingRegisterWrite(REG_HLD_DIR, direction);
}

/**
 * @brief Update Modbus data by finished command
 *
 * @param status STS status to output in Modbus status register
 */
void updateStatus(const STS status) {
  ModbusRTUServer.inputRegisterWrite(REG_INP_STS, status);
  ModbusRTUServer.coilWrite(COIL_NEW_STS, 1);
}

/**
 * @brief Complete Modbus command execution by full stop & status output
 *
 * @param status STS status of command execution
 */
void completeCommand(const STS status) {
  ModbusRTUServer.holdingRegisterWrite(REG_HLD_CMD, CMD_BRAKE);
  updateStatus(status);
}

/**
 * @brief Load controller data from EEPROM memory
 *
 * @return uint8_t exit code
 */
uint8_t loadFromMemory() {
  const uint8_t res = memory.begin(0, FIRMWARE_VERSION);

  if (0 == controllerData.updateRateHz) {
    controllerData.updateRateHz = defaultControllerData.updateRateHz;
  }
  ModbusRTUServer.holdingRegisterWrite(REG_HLD_RATE, controllerData.updateRateHz);

  for (byte i = 0; i < MOTOR_CNT; ++i) {
    StepperData& stepperData = controllerData.stepperData[i];
    const StepperData& defaultData = defaultControllerData.stepperData[i];

    if (0 == stepperData.gearRatio) {
      stepperData.gearRatio = defaultData.gearRatio;
    }
    if (0 == stepperData.wheelDiamMm) {
      stepperData.wheelDiamMm = defaultData.wheelDiamMm;
    }

    const byte baseIndex = 0 == i ? REG_HLD_SPD_L : REG_HLD_SPD_R;
    ModbusRTUServer.holdingRegisterWrite(baseIndex + 0, stepperData.maxSpeedMms); // 4 9
    ModbusRTUServer.holdingRegisterWrite(baseIndex + 1, stepperData.maxAccMms2 ); // 5 10
    ModbusRTUServer.holdingRegisterWrite(baseIndex + 2, stepperData.gearRatio  ); // 6 11
    ModbusRTUServer.holdingRegisterWrite(baseIndex + 3, stepperData.wheelDiamMm); // 7 12
    ModbusRTUServer.holdingRegisterWrite(baseIndex + 4, stepperData.isForward  ); // 8 13
  }

  return res;
}

/**
 * @brief Detect command by Modbus & execute it
 *
 */
void processDirection() {
  for (byte i = 0; i < MOTOR_CNT; ++i) {
    const StepperData& stepperData = controllerData.stepperData[i];
    switch (direction) {
      case DIR_STOP: {
        steppers[i].stop();
        break;
      }
      case DIR_FORWARD: {
        steppers[i].setSpeed(mmToSteps(stepperData, stepperData.maxSpeedMms));
        break;
      }
      case DIR_RIGHT: {
        steppers[i].setSpeed(
          mmToSteps(stepperData, stepperData.maxSpeedMms)
          * (i % 2 ? -1 : 1)
        );
        break;
      }
      case DIR_BACKWARD: {
        steppers[i].setSpeed(-mmToSteps(stepperData, stepperData.maxSpeedMms));
        break;
      }
      case DIR_LEFT: {
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

/**
 * @brief Detect command by Modbus & execute it
 *
 */
void processCommand() {
  const int packetReceived = ModbusRTUServer.poll();
  if (!packetReceived) {
    return; // no new packets
  }
  const int commandCoil = ModbusRTUServer.coilRead(COIL_NEW_CMD);
  if (!commandCoil) {
    return; // no new commands
  }
  // new command received
  ModbusRTUServer.coilWrite(COIL_NEW_CMD, 0);
  const long command = ModbusRTUServer.holdingRegisterRead(REG_HLD_CMD);
  switch (command) { // priority commands
    case CMD_BRAKE:
    case CMD_STOP:
    case CMD_MODE: {
      for (byte i = 0; i < MOTOR_CNT; ++i) {
        switch (command) {
          case CMD_BRAKE: // fall down
          case CMD_MODE: { steppers[i].brake(); break; }
          case CMD_STOP: { steppers[i].stop();  break; }
        }
      }

      if (CMD_MODE != command) {
        completeCommand(STS_OK);
        return;
      }

      direction = DIR_STOP;
      mode = (MODE)ModbusRTUServer.holdingRegisterRead(REG_HLD_MODE);
      STS status = STS_OK;
      switch (mode) {
        case MODE_TRG:
        case MODE_DRCT: { break; }
        // case MODE_WRD: only by hardware switch
        default: {
          mode = MODE_TRG;
          status = STS_ERR_MODE;
          ModbusRTUServer.holdingRegisterWrite(REG_HLD_MODE, mode);
          break;
        }
      }
      ModbusRTUServer.holdingRegisterWrite(REG_HLD_DIR, DIR_STOP);
      completeCommand(status);
      return;
    }
    case CMD_DIR: {
      direction = (DIR)ModbusRTUServer.holdingRegisterRead(REG_HLD_DIR);
      STS status = STS_OK;
      switch (direction) {
        case DIR_STOP:
        case DIR_FORWARD:
        case DIR_RIGHT:
        case DIR_BACKWARD:
        case DIR_LEFT: { break; }
        default: {
          direction = DIR_STOP;
          status = STS_ERR_DIR;
          ModbusRTUServer.holdingRegisterWrite(REG_HLD_DIR, DIR_STOP);
          break;
        }
      }

      processDirection();
      updateStatus(status);
    }
  }

  if (isSteppersMoving) {
    // moving now, ignore any others commands
    completeCommand(STS_ERR_MOVING);
    return;
  }

  // stopped now, process other commands
  switch (command) {
    case CMD_MOVE: {
      for (byte i = 0; i < MOTOR_CNT; ++i) {
        const int16_t target = ModbusRTUServer.holdingRegisterRead(REG_HLD_TARG_L + i);
        const StepperData& stepperData = controllerData.stepperData[i];
        steppers[i].setTarget(mmToSteps(stepperData, target));
      }
      completeCommand(STS_OK);
      break;
    };
    case CMD_UPDATE: {
      STS status = STS_OK;
      controllerData.updateRateHz = ModbusRTUServer.holdingRegisterRead(REG_HLD_RATE);
      if (0 == controllerData.updateRateHz) {
        status = STS_ERR_PARAMS;
      }

      for (byte i = 0; i < MOTOR_CNT; ++i) {
        StepperData& stepperData = controllerData.stepperData[i];
        const byte baseIndex = 0 == i ? REG_HLD_SPD_L : REG_HLD_SPD_R;
        stepperData.maxSpeedMms = ModbusRTUServer.holdingRegisterRead(baseIndex + 0); // 4 9
        stepperData.maxAccMms2  = ModbusRTUServer.holdingRegisterRead(baseIndex + 1); // 5 10
        stepperData.gearRatio   = ModbusRTUServer.holdingRegisterRead(baseIndex + 2); // 6 11
        stepperData.wheelDiamMm = ModbusRTUServer.holdingRegisterRead(baseIndex + 3); // 7 12
        stepperData.isForward   = ModbusRTUServer.holdingRegisterRead(baseIndex + 4); // 8 13

        if (0 == stepperData.gearRatio) {
          status = STS_ERR_PARAMS;
        }
        if (0 == stepperData.wheelDiamMm) {
          status = STS_ERR_PARAMS;
        }
      }

      if (STS_OK == status) {
        memory.updateNow();
      }
      loadFromMemory();
      initSteppers();

      completeCommand(status);
      break;
    };
    default: {
      completeCommand(STS_ERR_CMD);
      break;
    }
  }
}

/**
 * @brief Execute emergency stop motors
 *
 * @return bool return true if emergency pin raised
 */
bool processEmergency() {
  static bool isWasEmergency = false;
  const bool isEmergency = digitalRead(EMERGENCY_STOP_PIN);
  if (isEmergency && !isWasEmergency) {
    for (byte i = 0; i < MOTOR_CNT; ++i) {
      steppers[i].stop();
    }
    isWasEmergency = true;

    // drop active command
    ModbusRTUServer.coilWrite(COIL_NEW_CMD, 0);
    completeCommand(STS_EMRGENCY);
  } else if (!isEmergency && isWasEmergency) {
    isWasEmergency = false;
    updateStatus(STS_OK);
  }
  return isEmergency;
}

/**
 * @brief Process all signals from wired remote control
 *
 */
void processWires() {
  static MODE storedMode = mode;
  if (digitalRead(WRD_MODE_SWITCH_PIN)) {
    if (MODE_WRD != mode) {
      for (byte i = 0; i < MOTOR_CNT; ++i) {
        steppers[i].brake();
      }
      storedMode = mode;
      mode = MODE_WRD;
    }

    if (digitalRead(DIR_FORWARD_PIN)) {
      direction = DIR_FORWARD;
    } else if (digitalRead(DIR_RIGHT_PIN)) {
      direction = DIR_RIGHT;
    } else if (digitalRead(DIR_BACKWARD_PIN)) {
      direction = DIR_BACKWARD;
    } else if (digitalRead(DIR_LEFT_PIN)) {
      direction = DIR_LEFT;
    } else {
      direction = DIR_STOP;
    }
  } else {
    if (MODE_WRD == mode) {
      for (byte i = 0; i < MOTOR_CNT; ++i) {
        steppers[i].brake();
      }
      mode = storedMode;
    }
  }
}

/**
 * @brief Update steppers data in modbus registers
 *
 */
void updateCurrentStatus() {
  static unsigned long future = 0;
  const unsigned long now = millis();
  if (now < future) {
    return;
  }

  future = millis() + 1000 / controllerData.updateRateHz;

  // update current status
  for (byte i = 0; i < MOTOR_CNT; ++i) {
    const byte baseIndex = 0 == i ? REG_INP_STPR_L : REG_INP_STPR_R;
    const uint8_t stepperStatus =  steppers[i].getStatus();
    isSteppersMoving |= stepperStatus;
    ModbusRTUServer.inputRegisterWrite(baseIndex + 0, stepperStatus);

    const int16_t currentMm = stepsToMm(controllerData.stepperData[i], steppers[i].getCurrent());
    ModbusRTUServer.inputRegisterWrite(baseIndex + 1, currentMm);
  }
}

/**
 * @brief Check critical condition and stops next execution if it's true
 *
 * @param check condition
 * @param msg for print to Serial
 */
void checkCriticalError(const bool check, const char* msg) {
  if (!check) {
    Serial.println(msg);
    while (1);
  }
}

/**
 * @brief Main setup procedure, execute once by start
 *
 */
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  controllerData.stepperData[0].isForward = 0;

  checkCriticalError(
    ModbusRTUServer.begin(1, SERIAL_BAUD_RATE),
    "Failed to start Modbus RTU Server!"
  );
  checkCriticalError(
    ModbusRTUServer.configureCoils(COIL_START, COIL_END),
    "Failed to configure Coils"
  );
  checkCriticalError(
    ModbusRTUServer.configureInputRegisters(REG_INP_START, REG_INP_END),
    "Failed to configure Input Registers"
  );
  checkCriticalError(
    ModbusRTUServer.configureHoldingRegisters(REG_HLD_START, REG_HLD_END),
    "Failed to configure Holding Registers"
  );

  switch (loadFromMemory()) {
    case 0:  { break; } // ok, loaded from memory
    case 1:  { break; } // wrote default
    case 2:  { Serial.println("Memory: too large size"); while (1); }
    default: { Serial.println("Memory: unknown error");  while (1); }
  }

  mode = MODE_TRG;
  direction = DIR_STOP;

  ModbusRTUServer.inputRegisterWrite(REG_INP_VER, PROTOCOL_VERSION);

  initSteppers();
  updateStatus(STS_OK);

  // init wires
  pinMode(WRD_MODE_SWITCH_PIN, INPUT);
  pinMode(DIR_FORWARD_PIN,     INPUT);
  pinMode(DIR_RIGHT_PIN,       INPUT);
  pinMode(DIR_BACKWARD_PIN,    INPUT);
  pinMode(DIR_LEFT_PIN,        INPUT);

  pinMode(EMERGENCY_STOP_PIN,  INPUT);
}

/**
 * @brief Main loop of controller, infinite
 *
 */
void loop() {
  for (byte i = 0; i < MOTOR_CNT; ++i) {
    steppers[i].tick(); // process motors
  }

  updateCurrentStatus();
  if (processEmergency()) {
    return; // there is emergency, stop processing
  }

  processWires();

  switch (mode) {
    case MODE_TRG:
    case MODE_DRCT: {
      processCommand();
      break;
    }
    case MODE_WRD: {
      processDirection();
      break;
    }
    default: { break; }
  }

}

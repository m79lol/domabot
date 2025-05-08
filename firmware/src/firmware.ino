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

enum CMDS {
  CMD_BRAKE  = 0,
  CMD_STOP   = 1,
  CMD_MOVE   = 2,
  CMD_UPDATE = 3
};
enum STS {
  STS_OK = 0,
  STS_ERR_MOVING  = 1,
  STS_ERR_CMD     = 2,
  STS_ERR_PARAMS  = 3,
  STS_ERR_UNKNOWN = 99
};
enum COIL_ADDR {
  COIL_NEW_CMD = 0,
  COIL_NEW_STS = 1
};
enum REG_HLD {
  REG_HLD_CMD    = 0,
  REG_HLD_TARG_L = 1,   // starts from
  REG_HLD_TARG_R = 2,
  REG_HLD_RATE   = 3,
  REG_HLD_SPD_L  = 4, // starts from
  REG_HLD_SPD_R  = 9  // starts from
};
enum REG_INP_CTRL {
  REG_INP_VER    = 0,
  REG_INP_STS    = 1,
  REG_INP_STPR_L = 2, // starts from
  REG_INP_POS_L  = 3,
  REG_INP_STPR_R = 4,  // starts from
  REG_INP_POS_R  = 5
};

struct StepperData {
  uint16_t maxSpeedMms = 177;
  uint16_t maxAccMms2 = 55;
  uint16_t gearRatio = uint16_t(85.0 / 24.0 * 1000.0); // Z-driven / Z-leading * precision // 3541
  uint8_t wheelDiamMm = 200;
  uint8_t isForward = 1; // must zero for L motor, index 0
};

struct ControllerData {
  StepperData stepperData[MOTOR_CNT]; // left is 0, right is 1
  uint16_t updateRateHz = 50;
};
const ControllerData defaultControllerData;
ControllerData controllerData;
EEManager memory(controllerData);

bool isSteppersInited = false;

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

int32_t mmToSteps(const StepperData& stepperData, const int16_t value) {
  if (0 == stepperData.wheelDiamMm) {
    return 0;
  }
  return int32_t(
    double(value) / (double(stepperData.wheelDiamMm) * M_PI) 
      * (double(stepperData.gearRatio)/1000.0) * double(STEPS_REV)
  );
}

int16_t stepsToMm(const StepperData& stepperData, const int32_t value) {
  if (0 == STEPS_REV || 0 == stepperData.gearRatio) {
    return 0;
  }
  return int16_t(
    double(value) / double(STEPS_REV)
    / (double(stepperData.gearRatio)/1000.0) 
    * (double(stepperData.wheelDiamMm) * M_PI)
  );
}

void initSteppers() {
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

  ModbusRTUServer.holdingRegisterWrite(REG_HLD_CMD, 0);
}

void updateStatus(const STS status) {
  ModbusRTUServer.holdingRegisterWrite(REG_HLD_CMD, 0);
  ModbusRTUServer.inputRegisterWrite(REG_INP_STS, status);
  ModbusRTUServer.coilWrite(COIL_NEW_STS, 1);
}

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

void checkCriticalError(const bool check, const char* msg) {
  if (!check) {
    Serial.println(msg);
    while (1);
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  controllerData.stepperData[0].isForward = 0;

  checkCriticalError(
    ModbusRTUServer.begin(1, SERIAL_BAUD_RATE),
    "Failed to start Modbus RTU Server!"
  );
  checkCriticalError(
    ModbusRTUServer.configureCoils(COIL_NEW_CMD, 2),
    "Failed to configure Coils"
  );
  checkCriticalError(
    ModbusRTUServer.configureInputRegisters(REG_INP_VER, 2+2+2),
    "Failed to configure Input Registers"
  );
  checkCriticalError(
    ModbusRTUServer.configureHoldingRegisters(REG_HLD_CMD, (1+2+1+5*2)),
    "Failed to configure Holding Registers"
  );

  switch (loadFromMemory()) {
    case 0:  { break; } // ok, loaded from memory
    case 1:  { break; } // wrote default
    case 2:  { Serial.println("Memory: too large size"); while (1); }
    default: { Serial.println("Memory: unknown error");  while (1); }
  }

  ModbusRTUServer.inputRegisterWrite(REG_INP_VER, PROTOCOL_VERSION);
  
  initSteppers();
  updateStatus(STS_OK);
}

void loop() {
  for (byte i = 0; i < MOTOR_CNT; ++i) {
    steppers[i].tick();
  }

  static unsigned long future = 0;
  const unsigned long now = millis();
  uint8_t isSteppersMoving = 0;
  if (now > future) { // update current status
    for (byte i = 0; i < MOTOR_CNT; ++i) {
      const byte baseIndex = 0 == i ? REG_INP_STPR_L : REG_INP_STPR_R;
      const uint8_t stepperStatus =  steppers[i].getStatus();
      isSteppersMoving |= stepperStatus;
      ModbusRTUServer.inputRegisterWrite(baseIndex + 0, stepperStatus);

      int16_t currentMm = stepsToMm(controllerData.stepperData[i], steppers[i].getCurrent());
      ModbusRTUServer.inputRegisterWrite(baseIndex + 1, currentMm);
    }
    future = millis() + 1000 / controllerData.updateRateHz;
  }

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
  const long command = ModbusRTUServer.holdingRegisterRead(COIL_NEW_CMD);
  if (CMD_MOVE > command) { // only STOP & BRAKE commands has priority during movement
    for (byte i = 0; i < MOTOR_CNT; ++i) {
      switch (command) {
        case CMD_BRAKE: { steppers[i].brake(); break; }
        case CMD_STOP:  { steppers[i].stop();  break; }
      }
    }
    updateStatus(STS_OK);
    return;
  }

  if (isSteppersMoving) { 
    // moving now, ignore any others commands
    updateStatus(STS_ERR_MOVING);
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
      updateStatus(STS_OK);
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

      updateStatus(status);
      break;
    };
    default: {
      updateStatus(STS_ERR_CMD);
      break;
    }
  }

}

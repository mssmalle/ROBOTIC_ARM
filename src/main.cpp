#include <Arduino.h>
#include <SPI.h>
#include <StepperDriver.h>
#include <PololuMaestro.h>
#include <SoftwareSerial.h>
//change here

//Stepper Motor 1 Configuration - Shoulder Joint 1
const uint8_t Stepper1_CSPin = 33;
const uint8_t Stepper2_CSPin = 15;
const uint8_t Stepper1_ZeroPin = 32; //Digital 15
const uint8_t Stepper2_ZeroPin = 14; //Digital 15
const uint16_t CurrentLineLimit = 1680; //Set the Current Limit per phase of the stepper motor in Milliamps
const float CoilInductance = 3.2; //In mH
const float CoilVoltage = 2.8; //In V
const uint16_t Stepper1_StepsPerRev = 4000;
const uint16_t Stepper2_StepsPerRev = 200;

//Stepper Control Object Creation
Stepper Stepper1(Stepper1_CSPin, Stepper1_ZeroPin, CurrentLineLimit, HPSDStepMode::MicroStep1, Stepper1_StepsPerRev, CoilInductance, CoilVoltage); //init shoulder stepper with known configuration
Stepper Stepper2(Stepper2_CSPin, Stepper2_ZeroPin, CurrentLineLimit, HPSDStepMode::MicroStep64, Stepper2_StepsPerRev, CoilInductance, CoilVoltage); //init base stepper with known configuration

float degPerStep = 0.0;
float degreesToRotate = 90.0;
float rotVel = 45.0;

//function prototypes
void printResponse(stepper_response response);

void setup() {
  //Init STDOUT Serial for Error Messaging
  Serial.begin(115200);

  //Init Stepper Comms - Uses SPI
  SPI.begin();
  Stepper1.InitStepper();
  Stepper2.InitStepper();

  //stepper_response response = Stepper1.FindZeroAngle(); //Calibrate the stepper to abs 0 position on startup
  //printResponse(response);
}

void loop() {
  Stepper1.RotateDegrees(30.0);
  delay(1000);
  Stepper2.RotateDegrees(30.0);
  delay(1000);
  Stepper1.RotateDegrees(-30.0);
  delay(1000);
  Stepper2.RotateDegrees(-30.0);
  delay(3000);
}

void printResponse(stepper_response response) {
    switch(response) 
    {
      case ERROR_UNKNOWN:
        Serial.println("Error Unknown: Unable to Set Rotational Velocty");
        break;
      case WARNING_VEL_SET_ZERO:
        Serial.println("Warning: Velocity of the Stepper Motor has been set to 0.");
        break;
      case SUCCESS:
        Serial.println("Stepper Motor configuration was successfully updated.");
        break;
      case COMPLETED:
        Serial.println("Stepper Motor actuation successfully completed.");
        break;
      case TIMEOUT_ON_FIND_ZERO:
        Serial.println("Stepper Motor was unable to find its absolute zero angle. Make sure the zeroing switch is actuating properly.");
        break;
      case ERROR_ABOVE_VEL_MAX:
        Serial.println("Error: Stepper velocity set above max of 120 degrees per second. The stepper velocity has not been set.");
        break;
      case STALL:
        Serial.println("Critical Error: Stepper Motor Stall Detected. Immediatley shut off power!");
        break;
      default:
        break;
    }
}
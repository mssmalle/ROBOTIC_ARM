//Author: Max Smalley
//Date: 12/30/2020

#include "StepperDriver.h"

Stepper::Stepper(const uint8_t CSPin, const uint8_t ZeroPin, const uint16_t CurrentLineLimit, HPSDStepMode MicroStepMode, const uint16_t StepsPerRev, const float CoilInductance, const float CoilVoltage) {
	stepper_CSPin = CSPin;
	stepper_CurrentLimit = CurrentLineLimit;
	stepper_CoilInductance = CoilInductance;
	stepper_CoilVoltage = CoilVoltage;
	stepper_MicroStepMode = MicroStepMode;
	stepper_StepsPerRev = StepsPerRev;
	stepper_ZeroPin = ZeroPin;
	
	
	//Stepper Speed Variables
	stepper_RotationalVelocity = 45.0; //degrees per sec default value
	stepper_StepDelay = ConvertRotationalVelocityToStepDelay(stepper_RotationalVelocity);
}

///////////////////////////////////////Public Member Functions///////////////////////////////////
/** 
 * @Author: Max Smalley 
 * @Date: 2020-12-31
 * @Desc: Constructor for the Stepper class.
 * @Return: void
 */
void Stepper::InitStepper(void) {
	//configure stepper motor driver pin settings
	sd.setChipSelectPin(stepper_CSPin);
	//pinMode(stepper_ZeroPin, INPUT); //init zero switch input pin
	pinMode(14, INPUT);
	delay(1); 
	// Reset the driver to its default settings and clear latched status
	// conditions.
    sd.resetSettings();
	sd.clearStatus();

	// Select auto mixed decay.  TI's DRV8711 documentation recommends this mode
	// for most applications, and we find that it usually works well.
	sd.setDecayMode(HPSDDecayMode::AutoMixed);

	// Set the current limit. You should change the number here to an appropriate
	// value for your particular system.
	sd.setCurrentMilliamps36v4(stepper_CurrentLimit);

	// Set the number of microsteps that correspond to one full step.
	sd.setStepMode(stepper_MicroStepMode);

	sd.setDirection(0);

	// Enable the motor outputs.
	sd.enableDriver();	
}

/** 
 * @Author: Max Smalley 
 * @Date: 2020-12-31
 * @Desc: Set Stepper rotational veloctity based on the degrees/s input value. Microstep settings ARE considered
 * @Return: stepper_response - response status of rot velocity set request.
 */
stepper_response Stepper::SetRotationalVelocity(float degreesPerSec) {

	//precheck input vel param for unacceptable conditions
	if(degreesPerSec == 0.0) {
		return WARNING_VEL_SET_ZERO;
	} else if (degreesPerSec > MAX_ROT_VEL) {
		return ERROR_ABOVE_VEL_MAX;
	}

	//calculate desired rot vel in step delay
	unsigned int tempStepDelay = ConvertRotationalVelocityToStepDelay(degreesPerSec);
	if(tempStepDelay < 0.0) {
		return ERROR_UNKNOWN;
	}
	stepper_StepDelay = tempStepDelay;
	return SUCCESS;
}


/** 
 * @Author: Max Smalley 
 * @Date: 2020-12-31
 * @Desc: Rotates stepper motor N degrees at the current set velocity
 * @Return: stepper_response - Error or Success reponse 
 */
stepper_response Stepper::RotateDegrees(float degrees) {
	unsigned int ActualNumSteps;
	int sign = 1.0;
	
	//set rotation direction based on degree sign
	if (degrees > 0.0) {
		sd.setDirection(0);
		sign = 1.0;
		ActualNumSteps = ConvertDegreesToSteps(degrees);
	} else if (degrees < 0.0){
		sd.setDirection(1);
		sign = -1.0;
		ActualNumSteps = ConvertDegreesToSteps(-1.0*degrees);
	} else { //param passed is 0
		return COMPLETED;
	}
	
	/*NON BLOCKING IMPLIMENTATION - Setup an timer interrupt which runs the step function based on delay*/
	int stallStatus = 0;
	for(unsigned int x = 0; x < ActualNumSteps; x++){
		sd.step();
		stepper_JointAngle = (stepper_JointAngle + (2.0*sign*GetDegreesPerMicroStep())); //update current step angle
		delayMicroseconds(stepper_StepDelay); //step delay is set on init and can be changed dynamically to configure stepper motor velocity
	}
	return COMPLETED;
}

/** 
 * @Author: Max Smalley 
 * @Date: 2021-01-01
 * @Desc: Rotates stepper angle to absolute zero position. Position is found using servo actuated switch. Current position is also reset to 0 once the zero position is found
 * @Return: stepper_response - Returns response type based on success or error
 */
stepper_response Stepper::FindZeroAngle(void) {
	stepper_response response = ERROR_UNKNOWN;
	/* FUTURE IMPLIMENTATION - Add functionality to move servo arm up and down to expose zero switch.
	   Currently the switch is being manually placed at the zero position at startup */
	   
	SetRotationalVelocity(25.0); //set rot vel to responable value
	sd.setDirection(0); //always rotate counter clockwise or switch wont trip
	while(digitalRead(stepper_ZeroPin) == LOW){
		response = RotateDegrees(0.5);
	}
	stepper_JointAngle = 0.0;
	return response;
}

/** 
 * @Author: Max Smalley 
 * @Date: 2020-12-31
 * @Desc: Provides public access to current joint angle
 * @Return: float - current joint angle in degrees
 */
float Stepper::GetJointAngle(void) {
	return stepper_JointAngle;
}


/////////////////////////////////Protected Member Functions///////////////////////////////////
/** 
 * @Author: Max Smalley 
 * @Date: 2020-12-31
 * @Desc: Calculates degrees per step based on stepper config
 * @Return: float - number of degrees per full step
 */
float Stepper::GetDegreesPerFullStep(void) {
	return (360.0/(float)stepper_StepsPerRev);
}

/** 
 * @Author: Max Smalley 
 * @Date: 2020-12-31
 * @Desc: Calculates Maximum Theoretical Stepper Motor velocity
 * @Return: float - max velocity in deg/sec
 */
float Stepper::GetMaxStepperVelocity(void) {
	return (stepper_CoilVoltage/((stepper_CoilInductance*2.0*stepper_CurrentLimit)/((float)ConvertDegreesToSteps(360.0))));
}

/** 
 * @Author: Max Smalley 
 * @Date: 2020-12-31
 * @Desc: Calculates Degrees per microstep based on  current setting
 * @Return: float - degrees
 */
float Stepper::GetDegreesPerMicroStep(void) {
	return (GetDegreesPerFullStep()/((float)stepper_MicroStepMode));
}

////////////////////////////////////Private Functions//////////////////////////////////////////
/** 
 * @Author: Max Smalley 
 * @Date: 2020-12-31
 * @Desc: Converts input degrees to number of microsteps based on Stepper configuration
 * @Return: float - number of microsteps to be completed
 */
unsigned int Stepper::ConvertDegreesToSteps(float degrees) {
	float NumFullSteps = (degrees/GetDegreesPerFullStep());
	return (unsigned int)((NumFullSteps*((uint16_t)stepper_MicroStepMode)));///2.0
}

/** 
 * @Author: Max Smalley 
 * @Date: 2020-12-31
 * @Desc: Converts the input which is rotational velocity in degrees/sec to a step delay in microsecond
 * @Return: unsigned int - Converted step delay which will achieve desired velocity
 */
unsigned int Stepper::ConvertRotationalVelocityToStepDelay(float rotVel) {
	unsigned int StepsPerSec = ConvertDegreesToSteps(rotVel);
	float StepDelayUS = (1000000.0/(float)StepsPerSec);
	return (unsigned int)StepDelayUS;
}
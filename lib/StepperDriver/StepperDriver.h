//Author: Max Smalley
//Date: 12/30/2020

/*file: StepperDriver.h
//This is the main header file for the Stepper Driver library which provides a robotic arm control friendly interface on top of the Pololu Corp. DRV8711 Stepper Drive library which impliments settings and control for a single stepper motor. This library will extend that functionality to provide simple setup, specifc speed and angle control, and eventually angle control feedback using an encoder which is not yet implimented */

#ifndef _StepperDriver_H
#define _StepperDriver_H

#include <HighPowerStepperDriver.h>
using namespace std;

#define MAX_ROT_VEL 100.0


//stepper error reponses that can be returned during movement functions
typedef enum {
  STALL, COMPLETED, TIMEOUT_ON_FIND_ZERO, ERROR_UNKNOWN, WARNING_VEL_SET_ZERO, ERROR_ABOVE_VEL_MAX, SUCCESS
} stepper_response;

class Stepper {
	public:
		//constructor for the Stepper Class Object - params are all fundamental parameters of a bipolar stepper motor
		Stepper(const uint8_t CSPin, const uint8_t ZeroPin, const uint16_t CurrentLineLimit, HPSDStepMode MicroStepMode, const uint16_t StepsPerRev, const float CoilInductance, const float CoilVoltage);
				
		//Setup the DRV8711 Stepper Driver IC through SPI comms
		void InitStepper(void);
		
		//Function that will rotate the Stepper motor N degrees from its previous position
		stepper_response RotateDegrees(float degrees);
		
		//Function will set the stepper motor velocty in deg/sec
		stepper_response SetRotationalVelocity(float degreesPerSec);
		
		//Algorithm to calibrate stepper to absolute zero position on startup or stall
		stepper_response FindZeroAngle(void);
		
		//Public access to current joint angle
		float GetJointAngle(void);
	protected:
		//////////////////////Protected Member Variables///////////////////////////
		//Stepper Config Variables
		uint8_t stepper_CSPin;
		uint8_t stepper_ZeroPin;
		uint16_t stepper_CurrentLimit; //limit per phase of bipolar stepper motor
		float stepper_CoilInductance;	
		float stepper_CoilVoltage;	
		HPSDStepMode stepper_MicroStepMode;
		uint16_t stepper_StepsPerRev;
		
		//Stepper Position & Speed Variables
		float stepper_RotationalVelocity; //degrees per sec
		float stepper_StepDelay;
		float stepper_JointAngle;
		
		//////////////////////Protected Member Functions///////////////////////////

		float GetDegreesPerFullStep(void);//Get the number of degrees per step
		float GetDegreesPerMicroStep(void);
		float GetMaxStepperVelocity(void);
	private:		
		//Converts input degress to actual microsteps
		unsigned int ConvertDegreesToSteps(float degrees);
		unsigned int ConvertRotationalVelocityToStepDelay(float rotVel);
	public:
		//exposes the stepper driver object that allows for direct communication with DRV8711 comm library by Pololu Corp.
		//User typically should not need to access this unless they need to reconfigure stepper settings during runtime. 
		HighPowerStepperDriver sd;		
};

#endif
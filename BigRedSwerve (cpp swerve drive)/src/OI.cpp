/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"
#include "RobotMap.h"
#include "Commands/SwerveJoystickCommands/LazyFieldToggle.h"
#include "Commands/SwerveJoystickCommands/ZeroWheels.h"
#include <WPILib.h>

OI::OI() {
	// Process operator interface input here.
	leftStick.reset(new Joystick(0));
	rightStick.reset(new Joystick(1));

	JoystickButton* leftButton2 = new JoystickButton(leftStick.get(), 2);
	leftButton2->WhenPressed(new LazyFieldToggle());

	JoystickButton* leftButton3 = new JoystickButton(leftStick.get(), 3);
	leftButton3->WhenPressed(new ZeroWheels());

	/*JoystickButton* rightButton5 = new JoystickButton(rightStick.get(), 5);
	JoystickButton* rightButton3 = new JoystickButton(rightStick.get(), 3);

	JoystickButton* rightButton6 = new JoystickButton(rightStick.get(), 6);
	JoystickButton* rightButton4 = new JoystickButton(rightStick.get(), 4);
	*/
}

std::shared_ptr<Joystick> OI::GetLeftStick() {
	return leftStick;
}

std::shared_ptr<Joystick> OI::GetRightStick() {
	return rightStick;
}

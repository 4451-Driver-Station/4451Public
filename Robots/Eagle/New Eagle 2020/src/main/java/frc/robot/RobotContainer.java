/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...

	public final XboxController driverController = new XboxController(0);
	public final DriveTrainSubsystem drive = new DriveTrainSubsystem();

	private final JoystickButton buttonA = new JoystickButton(driverController, 1);
	private final JoystickButton buttonB = new JoystickButton(driverController, 2);
	private final JoystickButton buttonX = new JoystickButton(driverController, 3);
	private final JoystickButton buttonY = new JoystickButton(driverController, 4);

	public JoystickButton buttonList[] = new JoystickButton[4];

	public double getDriverForward() {
		return deadband(-driverController.getY(Hand.kLeft), .1);
	}

	public double getDriverTurn() {
		return deadband(driverController.getX(Hand.kRight), .1);
	}

	public void arcadeDrive(double fwd, double rot) {
		drive.arcadeDrive(fwd, rot);
	}

	public double getEncPos(int adr) {
		return drive.getEncoderPosition(adr);
	}

	public double getEncVel(int adr) {
		return drive.getEncoderVelocity(adr);
	}


	public double deadband(double input, double value) {
		if(Math.abs(input)<value) {
			return 0;
		} else {
			return input;
		}
	}
	/**
	 * The container for the robot.	Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();

		buttonList[0] = buttonA;
		buttonList[1] = buttonB;
		buttonList[2] = buttonX;
		buttonList[3] = buttonY;
	}

	/**
	 * Use this method to define your button->command mappings.	Buttons can be created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
	 * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
	}


	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An ExampleCommand will run in autonomous
		return null;
	}

	public boolean getButton(int buttonLetter) {
		return buttonList[buttonLetter].get();
	}
}

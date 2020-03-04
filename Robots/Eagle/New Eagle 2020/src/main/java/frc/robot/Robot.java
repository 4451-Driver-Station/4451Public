/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.ShuffleBoardSubsystem;


import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	
	private Command autoCommand;

	private RobotContainer rob;

	private ShuffleBoardSubsystem debug = new ShuffleBoardSubsystem("Debug");

	private double maxLeft = 0;
	private double maxRight = 0;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		// Instantiate our RobotContainer.  This will perform all our button bindings, and put our
		// autonomous chooser on the dashboard.
		rob = new RobotContainer();
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for items like
	 * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
	 *
	 * <p>This runs after the mode specific periodic functions, but before
	 * LiveWindow and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods.  This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();


	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 */
	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	/**
	 * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		autoCommand = rob.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (autoCommand != null) {
			autoCommand.schedule();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autoCommand != null) {
			autoCommand.cancel();
		}
		rob.drive.resetEncoders();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		if(rob.getButton(0)) {
			rob.drive.driveForward(.5);
		} else {
			runArcadeDrive();
		}
		putShuffleboardDebug();
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	private void runArcadeDrive() {
		double forward = rob.getDriverForward();
		double turn = rob.getDriverTurn();
		rob.arcadeDrive(forward, turn);
		
		//rob.drive.driveArcade(forward, turn);
		//rob.drive.driveArcade(forward, turn, rob.driverController.getY(Hand.kRight));
	}

	private void putShuffleboardDebug() {
		debug.put("Left Pos", rob.getEncPos(0));
		debug.put("Right Pos", rob.getEncPos(1));

		debug.put("Left Vel", rob.getEncVel(0));
		debug.put("Right Vel", rob.getEncVel(1));

		if(rob.getEncVel(0)>maxLeft) {
			maxLeft=rob.getEncVel(0);
		}
		if(rob.getEncVel(1)>maxRight) {
			maxRight=rob.getEncVel(1);
		}

		debug.put("MaxLeft", maxLeft);
		debug.put("MaxRight", maxRight);
	}
}

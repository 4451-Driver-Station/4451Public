/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveTrain;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.PigeonIMU;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class DriveTrainSubsystem extends SubsystemBase {
	/**
	 * Creates a new DriveTrain.
	 */
	
	WPI_TalonSRX leftTalon = new WPI_TalonSRX(DriveTrain.ID_LEFT_FRONT);
	WPI_VictorSPX leftVictor1 = new WPI_VictorSPX(DriveTrain.ID_LEFT_MID);
	WPI_VictorSPX leftVictor2 = new WPI_VictorSPX(DriveTrain.ID_LEFT_BACK);

	WPI_TalonSRX rightTalon = new WPI_TalonSRX(DriveTrain.ID_RIGHT_FRONT);
	WPI_VictorSPX rightVictor1 = new WPI_VictorSPX(DriveTrain.ID_RIGHT_MID);
	WPI_VictorSPX rightVictor2 = new WPI_VictorSPX(DriveTrain.ID_RIGHT_BACK);

	WPI_TalonSRX talonList[] = new WPI_TalonSRX[2];

	DifferentialDrive driveTrain = new DifferentialDrive(leftTalon, rightTalon);

	PigeonIMU gyro = new PigeonIMU(DriveTrain.ID_GYRO);

	public void arcadeDrive(double fwd, double rot) {
		driveTrain.arcadeDrive(fwd, rot);
	}

	public DriveTrainSubsystem() {
		talonList[0] = leftTalon;
		talonList[1] = rightTalon;

		leftTalon.setInverted(false);
		rightTalon.setInverted(true);
		rightVictor1.setInverted(true);
		rightVictor2.setInverted(true);

		leftVictor1.follow(leftTalon);
		leftVictor2.follow(leftTalon);
		rightVictor1.follow(rightTalon);
		rightVictor2.follow(rightTalon);

		leftTalon.setSensorPhase(true);
		rightTalon.setSensorPhase(true);

		driveTrain.setRightSideInverted(false); //this makes the two green lights on the controllers

		leftTalon.config_kP(0, DriveConstants.LEFT_PID_P);
		leftTalon.config_kI(0, DriveConstants.LEFT_PID_I);
		leftTalon.config_kD(0, DriveConstants.LEFT_PID_D);
		leftTalon.config_kF(0, DriveConstants.LEFT_PID_F);

		rightTalon.config_kP(0, DriveConstants.RIGHT_PID_P);
		rightTalon.config_kI(0, DriveConstants.RIGHT_PID_I);
		rightTalon.config_kD(0, DriveConstants.RIGHT_PID_D);
		rightTalon.config_kF(0, DriveConstants.RIGHT_PID_F);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void driveForward(double demandForward) {
		leftTalon.set(ControlMode.Velocity, DriveConstants.MAX_VELOCITY_TICK_SPEED*demandForward);
		rightTalon.set(ControlMode.Velocity, DriveConstants.MAX_VELOCITY_TICK_SPEED*demandForward);
	}

	public void driveArcade(double fwd, double rot) {
		double left = truncate(Math.pow(fwd, .5)+rot,1);
		double right = truncate(Math.pow(fwd, .5)+rot,1);
		leftTalon.set(ControlMode.Velocity, DriveConstants.MAX_VELOCITY_TICK_SPEED*left);
		rightTalon.set(ControlMode.Velocity, DriveConstants.MAX_VELOCITY_TICK_SPEED*right);
	}

	//caution is how much the robot slows down when turning
	public void driveArcade(double fwd, double rot, double caution) {
		double left = truncate(Math.pow(fwd, .5)+rot,1)/Math.abs(caution)+1;
		double right = truncate(Math.pow(fwd, .5)+rot,1)/Math.abs(caution)+1;
		leftTalon.set(ControlMode.Velocity, DriveConstants.MAX_VELOCITY_TICK_SPEED*left);
		rightTalon.set(ControlMode.Velocity, DriveConstants.MAX_VELOCITY_TICK_SPEED*right);
	}

	//this is purely used in driveArcade()
	private double truncate(double input, double limit) {
		if(Math.abs(input)>limit) {
			return limit;
		} else {
			return input;
		}
	}

	public int getEncoderPosition(WPI_TalonSRX talon) {
		return talon.getSelectedSensorPosition();
	}

	/**
	 * @param talon 0: left Front, 1: left Back, 2: right Front, 3: right Back
	 */

	public int getEncoderPosition(int talon) {
		return talonList[talon].getSelectedSensorPosition();
	}

	public int getEncoderVelocity(WPI_TalonSRX talon) {
		return talon.getSelectedSensorVelocity();
	}

	/**
	 * @param falcon 0: left Front, 1: left Back, 2: right Front, 3: right Back
	 */

	public int getEncoderVelocity(int falcon) {
		return talonList[falcon].getSelectedSensorVelocity();
	}
	
	public void resetEncoders() {
		leftTalon.setSelectedSensorPosition(0);
		rightTalon.setSelectedSensorPosition(0);
	}

	public void resetSensors() {
		resetEncoders();
		gyro.setYaw(0);
	}
}

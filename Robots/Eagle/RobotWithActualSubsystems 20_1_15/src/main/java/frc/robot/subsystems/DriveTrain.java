/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.controller.PIDController;

public class DriveTrain extends SubsystemBase {

  public WPI_TalonSRX frontLeftTalon = new WPI_TalonSRX(2);
  public WPI_TalonSRX frontRightTalon = new WPI_TalonSRX(1);

	WPI_VictorSPX leftSlave1 = new WPI_VictorSPX(4);
	WPI_VictorSPX rightSlave1 = new WPI_VictorSPX(3);
	WPI_VictorSPX leftSlave2 = new WPI_VictorSPX(6);
  WPI_VictorSPX rightSlave2 = new WPI_VictorSPX(5);

  PigeonIMU gyro = new PigeonIMU(0);

  public DifferentialDrive drive = new DifferentialDrive(frontLeftTalon, frontRightTalon);
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24.5));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  Pose2d pose;

  public SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1.32, 2.38, 0.867);

  PIDController leftPIDController = new PIDController(21.9, 0, 0);
  PIDController rightPIDController = new PIDController(21.9, 0, 0);
  
  public DriveTrain() {
    frontLeftTalon.setInverted(false);
    frontRightTalon.setInverted(false);

    leftSlave1.follow(frontLeftTalon);
    leftSlave2.follow(frontLeftTalon);
    rightSlave1.follow(frontRightTalon);
    rightSlave2.follow(frontRightTalon);

    leftSlave1.setInverted(InvertType.FollowMaster);
    leftSlave2.setInverted(InvertType.FollowMaster);
    rightSlave1.setInverted(InvertType.FollowMaster);
    rightSlave2.setInverted(InvertType.FollowMaster);

    
  }

	public Rotation2d getHeading() {
		double[] ypr = new double[3];
		gyro.getYawPitchRoll(ypr);
    return Rotation2d.fromDegrees(Units.degreesToRadians(-ypr[0]));
  }

  public double getGyroValue() {
		double[] ypr = new double[3];
		gyro.getYawPitchRoll(ypr);
    return Rotation2d.fromDegrees(-ypr[0]).getDegrees();
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      frontLeftTalon.getSelectedSensorVelocity() /4096D*6*Math.PI*.0254*10,
	  	frontRightTalon.getSelectedSensorVelocity() /4096D*6*Math.PI*.0254*10
	  );
  }
  
  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }
  public PIDController getLeftPIDController() {
    return leftPIDController;
  }
  public PIDController getRightPIDController() {
    return rightPIDController;
  }
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  public void setOutput(double leftVolts, double rightVolts) {
    frontLeftTalon.set(leftVolts / 12D);
    frontRightTalon.set(rightVolts / 12D);
  }

  public void resetGyro() {
    gyro.setYaw(0);
  }

  public void resetEncoders() {
    frontLeftTalon.setSelectedSensorPosition(0);
    frontRightTalon.setSelectedSensorPosition(0);
  }

  public void resetPose() {
    resetGyro();
    resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(
      getHeading(),
      frontLeftTalon.getSelectedSensorPosition(),
      frontRightTalon.getSelectedSensorPosition()
    );
  }
}

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
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

public class DriveTrain extends SubsystemBase {

  public WPI_TalonSRX frontLeftTalon = new WPI_TalonSRX(2);
  public WPI_TalonSRX frontRightTalon = new WPI_TalonSRX(1);

	WPI_VictorSPX leftSlave1 = new WPI_VictorSPX(4);
	WPI_VictorSPX rightSlave1 = new WPI_VictorSPX(3);
	WPI_VictorSPX leftSlave2 = new WPI_VictorSPX(6);
  WPI_VictorSPX rightSlave2 = new WPI_VictorSPX(5);

  public PigeonIMU gyro = new PigeonIMU(0);

  public DifferentialDrive drive = new DifferentialDrive(frontLeftTalon, frontRightTalon);
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24.24));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  Pose2d pose;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1.19, 2.3, 0.906);

  public DriveTrain() {
    leftSlave1.follow(frontLeftTalon);
    leftSlave2.follow(frontLeftTalon);
    rightSlave1.follow(frontRightTalon);
    rightSlave2.follow(frontRightTalon);

    frontLeftTalon.setInverted(false);
		frontRightTalon.setInverted(true);
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
  
  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public Pose2d getPose() {
    return pose;
  }

  public double leftValue = 0;
  public double rightValue = 0;
  public void setOutput(double leftVelocity, double rightVelocity) {
    var leftAccel = (leftVelocity - stepsPerDesisecToMetersPerSecond(frontLeftTalon.getSelectedSensorVelocity()))/20;
    var rightAccel = (leftVelocity - stepsPerDesisecToMetersPerSecond(frontRightTalon.getSelectedSensorVelocity()))/20;
    //YOU MIGHT NEED TO NOT DIVIDE THIS ^ BY 20
    var leftFeedForwardVolts = feedforward.calculate(leftVelocity, leftAccel);
    var rightFeedForwardVolts = feedforward.calculate(rightVelocity, rightAccel);

    frontLeftTalon.set(ControlMode.Velocity, metersPerSecondToStepsPerDesisec(leftVelocity), DemandType.ArbitraryFeedForward, leftFeedForwardVolts / 12);
    frontRightTalon.set(ControlMode.Velocity, metersPerSecondToStepsPerDesisec(rightVelocity), DemandType.ArbitraryFeedForward, rightFeedForwardVolts / 12);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(
      getHeading(),
      stepsToMeters(frontLeftTalon.getSelectedSensorPosition()),
      stepsToMeters(frontRightTalon.getSelectedSensorPosition()));
  }

  public double stepsToMeters(double input) {
    return input/4096*(6*Math.PI)*.0254;
  }

  public double stepsPerDesisecToMetersPerSecond(double input) {
    return input/4096*(6*Math.PI)*.0254*10;
  }

  public double metersPerSecondToStepsPerDesisec(double input) {
    return input*4096/(6*Math.PI)/.0254/10;
  }

}
;

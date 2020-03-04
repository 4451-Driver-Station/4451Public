/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.util.Units;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */
  WPI_TalonSRX leftTalon = new WPI_TalonSRX(2);
  WPI_TalonSRX rightTalon = new WPI_TalonSRX(1);
  WPI_VictorSPX leftSlave1 = new WPI_VictorSPX(4);
  WPI_VictorSPX rightSlave1 = new WPI_VictorSPX(3);
  WPI_VictorSPX leftSlave2 = new WPI_VictorSPX(6);
  WPI_VictorSPX rightSlave2 = new WPI_VictorSPX(5);

  DifferentialDrive driveTrain = new DifferentialDrive(leftTalon, rightTalon);

  PigeonIMU gyro = new PigeonIMU(0);

  //Kinematics and Odometry
    
  DifferentialDriveOdometry odometry;

  //SUBSYSTEM METHODS ================================================================================================

  public DriveTrain() {
    //follow Victors to Talons
    leftSlave1.follow(leftTalon);
    leftSlave2.follow(leftTalon);
    rightSlave1.follow(rightTalon);
    rightSlave2.follow(rightTalon);

    leftTalon.setInverted(true);
    
    //reset Encoders before anything
    resetEncoders();
    zeroHeading();

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    odometry.update(Rotation2d.fromDegrees(getHeading()), getMotorDistance(leftTalon), getMotorDistance(rightTalon));
  }

  //COMMANDS ================================================================================================

  public void arcadeDrive(double fwd, double rot) {
    driveTrain.arcadeDrive(fwd, rot);
  }

  public void setVolts(double leftVolts, double rightVolts) {
    leftTalon.set(leftVolts / 12D);
    rightTalon.set(rightVolts / 12D);
  }

  public void setMaxOutput(double max) {
    driveTrain.setMaxOutput(max);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      getEncSpeed(leftTalon),
      getEncSpeed(rightTalon)
    );
  }

  public void ResetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public int getEncCounts(WPI_TalonSRX motor) {
    return motor.getSelectedSensorPosition();
  }

  public int getEncSpeed(WPI_TalonSRX motor) {
    return motor.getSelectedSensorPosition();
  }

  public double getMotorDistance(WPI_TalonSRX motor) {
    return getEncCounts(motor)*Constants.EncToMeters;
  }

  public void resetEncoders() {
    leftTalon.setSelectedSensorPosition(0);
    rightTalon.setSelectedSensorPosition(0);
  }

  public void zeroHeading() {
    gyro.setYaw(0);
  }

  public double getHeading() {
    double[] ypr = new double[3];
    gyro.getYawPitchRoll(ypr);
    return Math.IEEEremainder(ypr[0], 360)*-1;
  }
}

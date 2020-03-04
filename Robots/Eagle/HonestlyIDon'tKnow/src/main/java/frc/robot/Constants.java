/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //Encoder counts to Meters conversion factor
    public static final double EncToMeters = 1/4096*Units.inchesToMeters(6)*Math.PI;



    public static final double ks_Volts = 1.32;
    public static final double kv_VoltSecondsPerMeter = 2.38;
    public static final double ka_VoltSecondsSquaredPerMeter = 0.867;
    public static final double kP_DriveVel = 21.9;
    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.7;
    public static final double kTrackWidthmeters = Units.inchesToMeters(24.5);
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthmeters);
    public static final double autoMaxOutputVoltage = 10;







}

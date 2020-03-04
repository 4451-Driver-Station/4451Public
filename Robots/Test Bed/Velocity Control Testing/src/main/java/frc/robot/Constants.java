/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final int TICKS_PER_ROTATION = 4096;
	public static final int MAX_SELECTED_TICKSPEED = 35000;

	public static double P = 0.001;
	public static final double I = 0;
	public static final double D = 0;

	public static double F_RAW_GAIN_AUTO(double percentInput) {
		switch(Double.toString(percentInput)) {
			case ".1": return 2500;
			case ".5": return 17450;
			case "1": return 36500;
			default: return 0;
		}
	}

	public static double F_RAW_GAIN_TELE(double percentInput) {
		return (2061.85*Math.pow(percentInput, 3)-147.619)+33891.6*percentInput;
	}

	public static double F_AUTO(double percentPower) {
		return (percentPower*1023)/F_RAW_GAIN_AUTO(percentPower);
	}

	public static double F_TELE(double percentPower) {
		return (percentPower*1023)/F_RAW_GAIN_TELE(percentPower);
	}

	public static final int CRUISE_VELOCITY = MAX_SELECTED_TICKSPEED;
	public static final int MAX_ACCELERATION = MAX_SELECTED_TICKSPEED;

}

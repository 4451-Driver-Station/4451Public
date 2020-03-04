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

		public final static class DriveTrain {

			//UPDATE THESE!
			public static final int ID_LEFT_FRONT = 2;
			public static final int ID_LEFT_MID = 4;
			public static final int ID_LEFT_BACK = 6;
			public static final int ID_RIGHT_FRONT = 1;
			public static final int ID_RIGHT_MID = 3;
			public static final int ID_RIGHT_BACK = 5;

			public static final int ID_GYRO = 0;

		}

		public static final class DriveConstants {

			public static final int ENC_TICKS_PER_ROTATION = 4096;

			public static final int MAX_VELOCITY_TICK_SPEED = 3500;

			public static final double LEFT_PID_P = .5;
			public static final double LEFT_PID_I = 0;
			public static final double LEFT_PID_D = 0;
			public static final double LEFT_PID_F = (1023/MAX_VELOCITY_TICK_SPEED);

			public static final double RIGHT_PID_P = .5;
			public static final double RIGHT_PID_I = 0;
			public static final double RIGHT_PID_D = 0;
			public static final double RIGHT_PID_F = (1023/MAX_VELOCITY_TICK_SPEED);
			
		}
}

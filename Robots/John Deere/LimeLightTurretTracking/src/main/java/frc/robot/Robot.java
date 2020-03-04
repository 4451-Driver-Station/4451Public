/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The SixTalonArcadeDrive example demonstrates the ability to create WPI Talons/Victors
 * to be used with WPI's drivetrain classes. WPI Talons/Victors contain all the functionality
 * of normally created Talons/Victors (Phoenix) with the remaining SpeedController functions
 * to be supported by WPI's classes. 
 * 
 * The example uses two master motor controllers passed into WPI's DifferentialDrive Class 
 * to control the remaining 4 Talons (Follower Mode) to provide a simple Tank Arcade Drive 
 * configuration.
 *
 * Controls:
 * Left Joystick Y-Axis: Drive robot in forward and reverse direction
 * Right Joystick X-Axis: Turn robot in right and left direction
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import java.lang.Math;



public class Robot extends TimedRobot {
	/* Master Talons for arcade drive */
	private WPI_VictorSPX turretMotor = new WPI_VictorSPX(2);

    /* Construct drivetrain by providing master motor controllers 
	private DifferetialDrive drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
	*/ 

	XboxController controller = new XboxController(0);
	JoystickButton aButton = new JoystickButton(controller, 1);

	edu.wpi.first.networktables.NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	
	@Override
	public void teleopInit() {
		super.teleopInit();
		
	}
		
	@Override
	public void teleopPeriodic() {
	
		NetworkTableEntry targetv = table.getEntry("tv");
		if (aButton.get()==true) {
			table.getEntry("pipeline").setNumber(1);
			if (targetv.getDouble(0) >= 1) {
				NetworkTableEntry targetx = table.getEntry("tx");
				double tx = targetx.getDouble(0);
				turretMotor.set(tx/-20);
			} else {
			double turn = 0.3*Math.abs(controller.getX(Hand.kLeft)) < 0.2 ? 0 : -controller.getX(Hand.kLeft);
			turretMotor.set(turn);
			}	
		} else {
			table.getEntry("pipeline").setNumber(0);
			double turn = 0.5*Math.abs(controller.getX(Hand.kLeft)) < 0.2 ? 0 : -controller.getX(Hand.kLeft);
			turretMotor.set(turn);
		}
	}
	/*
	//Limelight table
	double Kp = -0.1f;
	
  
	
	
	  double aim_error = tx;
	  double steering_adjust = Kp*aim_error;
	  double AimMinCmd = 0.095f;
	  WPI_TalonSRX left_command1 = Left1, Left2;
	  WPI_TalonSRX right_command1 = Right1, Right2;	
	
	//Drivesticks
	turretMotor((left_command* .7), (right_command* .75));
	
	
	//Buttons
	
	 //Vision code stuff
	if (controller.get()){
		double heading_error = tx;
		steering_adjust = Kp * tx;
		
		left_command1 += steering_adjust; //where I get the error
		right_command1 -= steering_adjust; //where I get the error
		
	}


*/
}

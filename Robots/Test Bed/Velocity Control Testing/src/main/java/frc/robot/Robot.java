/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.LidarLitePWM;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {

	private WPI_TalonSRX TheOneMotor = new WPI_TalonSRX(0);
	private XboxController controller = new XboxController(0);
	private JoystickButton joyA = new JoystickButton(controller, 1);
	private JoystickButton joyB = new JoystickButton(controller, 2);
	private JoystickButton joyX = new JoystickButton(controller, 3);
	private JoystickButton joyY = new JoystickButton(controller, 4);

	private Command m_autonomousCommand;

	private RobotContainer rob;


	private double start = 0;
	private double end1 = 0;

	private double temp = 0;

	private boolean x_c = false;
	private boolean x_l = false;
	private boolean y_c = false;
	private boolean y_l = false;

	private double temp2 = 0;


	NetworkTableInstance defaultInstance;

	private double getTime() {
		return Timer.getFPGATimestamp();
	}

	private double getTimeSinceStart() {
		return Timer.getFPGATimestamp()-start;
	}
	
	private DigitalInput lidarInput = new DigitalInput(0);

	private LidarLitePWM lidar = new LidarLitePWM(lidarInput);

	private double lidarReadings[] = new double[20];
	/*
	private double updateLidarReadings(double currentInput) {
		for(int i = 0; i < 20; i++) {
			return null;
		}
	}
	*/

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		// Instantiate our RobotContainer.	This will perform all our button bindings, and put our
		// autonomous chooser on the dashboard.
		rob = new RobotContainer();

		defaultInstance = NetworkTableInstance.getDefault();

		TheOneMotor.config_kP(0, Constants.P);
		TheOneMotor.config_kI(0, Constants.I);
		TheOneMotor.config_kD(0, Constants.D);
		
		TheOneMotor.config_kF(0, Constants.F_AUTO(.5));

		//configure motion magic variables

		TheOneMotor.configMotionAcceleration(Constants.MAX_ACCELERATION);
		TheOneMotor.configMotionCruiseVelocity(Constants.CRUISE_VELOCITY);

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
		// Runs the Scheduler.	This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods.	This must be called from the robot's periodic
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
		m_autonomousCommand = rob.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}

		start = getTime();
		TheOneMotor.setSelectedSensorPosition(0);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {

		double time = getTimeSinceStart();

		if(time<5) {
			TheOneMotor.set(0);
		} else {
			if(TheOneMotor.getSelectedSensorPosition()<99.95*Constants.TICKS_PER_ROTATION && time<5+15) {
				TheOneMotor.set(ControlMode.MotionMagic, 100*Constants.TICKS_PER_ROTATION);
			} else {
				if(end1 == 0) {end1 = time;}
				if(time<2+end1) {
					TheOneMotor.set(0);
				} else {
					if(TheOneMotor.getSelectedSensorPosition()>0.05*Constants.TICKS_PER_ROTATION && time<end1+2+15) {
						TheOneMotor.set(ControlMode.MotionMagic, 0*Constants.TICKS_PER_ROTATION);
					} else {
						TheOneMotor.set(0);
					}
				}
			}
		}

		defaultInstance.getTable("Debug Graphs").getEntry("Velocity").setNumber(TheOneMotor.getSelectedSensorVelocity());
		defaultInstance.getTable("Debug Graphs").getEntry("Position").setNumber(TheOneMotor.getSelectedSensorPosition()/Constants.TICKS_PER_ROTATION);
		
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		TheOneMotor.setSelectedSensorPosition(0);
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {

		double forward = bandwidth(-controller.getY(Hand.kLeft), .075);

		x_l = x_c;
		y_l = y_c;
		x_c = joyX.get();
		y_c = joyY.get();

		if(x_c && !x_l) {
			temp -= .1;
		}
		if(y_c && !y_l) {
			temp += .1;
		}

		if(joyY.get()) {
			temp2 += .0001;
		}

		if(joyX.get()) {
			temp2 -= .0001;
		}

		if(joyA.get()) {
			TheOneMotor.config_kF(0, Constants.F_TELE(.5));
			TheOneMotor.set(ControlMode.Velocity, 100*forward*Constants.MAX_SELECTED_TICKSPEED);

			//TheOneMotor.set(ControlMode.MotionMagic, 10*Constants.TICKS_PER_ROTATION);
		} else {
		TheOneMotor.set(0);
		}
		if(joyB.get()) {
			TheOneMotor.setSelectedSensorPosition(0);
		}

		defaultInstance.getTable("Debug Graphs").getEntry("Velocity").setNumber(TheOneMotor.getSelectedSensorVelocity());
		defaultInstance.getTable("Debug Graphs").getEntry("Position").setNumber(TheOneMotor.getSelectedSensorPosition());
		defaultInstance.getTable("Debug Graphs").getEntry("Forward").setNumber(controller.getY(Hand.kLeft));

		defaultInstance.getTable("Debug Graphs").getEntry("LidarDistance").setNumber(lidar.getDistance());

		smartDashboardPut();
	}

	private double bandwidth(double input, double limit) {
		if(Math.abs(input)<limit) {
			return 0;
		} else {
			return input;
		}
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

	private void smartDashboardPut() {
		Number motor[]= new Number[2];

		motor[0] = TheOneMotor.getSelectedSensorPosition();
		motor[1] = TheOneMotor.getSelectedSensorVelocity();

		defaultInstance.getTable("Debug Graphs").getEntry("B").setNumberArray(motor);

	}
}

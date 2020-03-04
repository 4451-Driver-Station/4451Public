package frc.robot;

import static frc.robot.OI.*;
import static frc.robot.RobotMap.*;

import java.lang.reflect.Field;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.commands.LimeDrive.Mode;
import frc.robot.sensors.LimeLight.LEDMode;

public class Robot extends TimedRobot {

    public static Robot instance;

    // drive commands
    private final TeleopDrive teleopDrive = new TeleopDrive();
    private final frc.robot.commands.LimeDrive targetDrive = new frc.robot.commands.LimeDrive(Mode.Hatch);
    private final frc.robot.commands.LimeDrive ballDrive = new frc.robot.commands.LimeDrive(Mode.Ball);
    private final BumpCommand bumpLeftCommand = new BumpCommand(true);
    private final BumpCommand bumpRightCommand = new BumpCommand(false);
    private final BackupCommand backupCommand = new BackupCommand(-1500);
    private final BackupCommand backupLongCommand = new BackupCommand(-48000);
    // elevator commands
    private final ManualElevatorCommand manualElevatorCommand = new ManualElevatorCommand();
    private final ElevatorCommand home = new ElevatorCommand(0); // home
    private final ElevatorCommand position1 = new ElevatorCommand(21000); // hatch 2
    private final ElevatorCommand position2 = new ElevatorCommand(41700); // hatch 3
    private final ElevatorCommand position3 = new ElevatorCommand(6000); // ball 1
    private final ElevatorCommand position4 = new ElevatorCommand(26000); // ball 2
    private final ElevatorCommand position5 = new ElevatorCommand(45000); // ball 3
    // climb commands
    private final SwitchCommandsCommand switchCommandsCommand = new SwitchCommandsCommand();
    private final ManualClimbCommand manualClimbCommand = new ManualClimbCommand();
    // scoring commands
    private final IntakeCommand intakeCommand = new IntakeCommand();
    private final HatchCupControl hatchDownCommand = new HatchCupControl(true);
    private final HatchCupControl hatchUpCommand = new HatchCupControl(false);
    private final ScoringMechanismControl scoringMechanismControl = new ScoringMechanismControl();
    private final HatchStickControl hatchStickControl = new HatchStickControl();

    public static boolean climbMode = false;

    @Override
    public void robotInit() {
        instance = this;

        // setup talons
        setupTalon(leftFront, true);
        setupTalon(rightFront, false);

        // set invertion
        leftFront.setInverted(false);
        rightFront.setInverted(true);
        leftMiddle.setInverted(false);
        rightMiddle.setInverted(true);
        leftRear.setInverted(false);
        rightRear.setInverted(true);

        // let the victors follow the talons
        leftMiddle.follow(leftFront);
        leftRear.follow(leftFront);
        rightMiddle.follow(rightFront);
        rightRear.follow(rightFront);

        // setup up elevator pid
        elevator.setInverted(true);
        elevator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 100);
        elevator.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10);
        elevator.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);
        elevator.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms, 10);
        elevator.configVelocityMeasurementWindow(16, 10);
        elevator.configNominalOutputForward(0, 100);
        elevator.configNominalOutputReverse(0, 100);
        elevator.configPeakOutputForward(1, 100);
        elevator.configPeakOutputReverse(-1, 100);
        elevator.selectProfileSlot(0, 0);
        elevator.config_kF(0, config.pids.elevator.f, 100);
        elevator.config_kP(0, config.pids.elevator.p, 100);
        elevator.config_kI(0, config.pids.elevator.i, 100);
        elevator.config_kD(0, config.pids.elevator.d, 100);
        elevator.setSensorPhase(config.pids.elevator.sensorPhase);
        elevator.setSelectedSensorPosition(0);
        elevator.configMotionCruiseVelocity(config.pids.elevator.cruiseVelocity);
        elevator.configMotionAcceleration(config.pids.elevator.acceleration);

        // SmartDashboard.putNumber("Left_P0", RobotMap.leftTalon_P0);
        // SmartDashboard.putNumber("Left_I0", RobotMap.leftTalon_I0);
        // SmartDashboard.putNumber("Left_D0", RobotMap.leftTalon_D0);
        // SmartDashboard.putNumber("Left_F0", RobotMap.leftTalonFeedForward0);
        // SmartDashboard.putNumber("Right_P0", RobotMap.rightTalon_P0);
        // SmartDashboard.putNumber("Right_I0", RobotMap.rightTalon_I0);
        // SmartDashboard.putNumber("Right_D0", RobotMap.rightTalon_D0);
        // SmartDashboard.putNumber("Right_F0", RobotMap.rightTalonFeedForward0);

        // SmartDashboard.putNumber("Left_P1", RobotMap.leftTalon_P1);
        // SmartDashboard.putNumber("Left_I1", RobotMap.leftTalon_I1);
        // SmartDashboard.putNumber("Left_D1", RobotMap.leftTalon_D1);
        // SmartDashboard.putNumber("Left_F1", RobotMap.leftTalonFeedForward1);
        // SmartDashboard.putNumber("Right_P1", RobotMap.rightTalon_P1);
        // SmartDashboard.putNumber("Right_I1", RobotMap.rightTalon_I1);
        // SmartDashboard.putNumber("Right_D1", RobotMap.rightTalon_D1);
        // SmartDashboard.putNumber("Right_F1", RobotMap.rightTalonFeedForward1);
    }

    @Override
    public void robotPeriodic() {
        // put different stats on the dashboard
        SmartDashboard.putNumber("Lidar Left", lidarLeft.getInches());
        SmartDashboard.putNumber("Lidar Right", lidarRight.getInches());
        SmartDashboard.putNumber("Lidar Front", lidarFront.getInches());
        SmartDashboard.putNumber("Lidar Back", lidarBack.getInches());
        SmartDashboard.putBoolean("LimeLight Has Target", limeLight.hasTarget());
        if (limeLight.hasTarget()) {
            SmartDashboard.putNumber("LimeLight X", limeLight.getX());
            SmartDashboard.putNumber("LimeLight Y", limeLight.getY());
        } else {
            SmartDashboard.putNumber("LimeLight X", 0);
            SmartDashboard.putNumber("LimeLight Y", 0);
        }
        SmartDashboard.putBoolean("Ball Limitswitch", !ballLimitSwitch.get());
        SmartDashboard.putBoolean("Hatch Released", hatchButton.get());
        SmartDashboard.putBoolean("Climb Mode", climbMode);

        var table = NetworkTableInstance.getDefault().getTable("Debug");
        // table.getEntry("Talon Left
        // Voltage").setNumber(leftFront.getMotorOutputVoltage());
        // table.getEntry("Talon Right
        // Voltage").setNumber(rightFront.getMotorOutputVoltage());
        // table.getEntry("Victor Middle Left
        // Voltage").setNumber(leftMiddle.getMotorOutputVoltage());
        // table.getEntry("Victor Middle Right
        // Voltage").setNumber(rightMiddle.getMotorOutputVoltage());
        // table.getEntry("Victor Back Left
        // Voltage").setNumber(leftRear.getMotorOutputVoltage());
        // table.getEntry("Victor Back Right
        // Voltage").setNumber(rightRear.getMotorOutputVoltage());

        // table.getEntry("Talon Left Current").setNumber(leftFront.getOutputCurrent());
        // table.getEntry("Talon Right
        // Current").setNumber(rightFront.getOutputCurrent());

        table.getEntry("Elevator Current").setNumber(elevator.getOutputCurrent());
        table.getEntry("Elevator Voltage").setNumber(elevator.getMotorOutputVoltage());
        table.getEntry("Elevator Ticks").setNumber(elevator.getSelectedSensorPosition());

        // table.getEntry("PDP Voltage").setNumber(pdp.getVoltage());
        // table.getEntry("PDP Current").setNumber(pdp.getTotalCurrent());
        // table.getEntry("Controller Speed").setNumber(OI.getSpeed());
        table.getEntry("Left Error").setNumber(leftFront.getClosedLoopError());
        table.getEntry("Right Error").setNumber(rightFront.getClosedLoopError());
        // table.getEntry("Left Actual
        // Speed").setNumber(leftFront.getSensorCollection().getPulseWidthVelocity());
        // table.getEntry("Right Actual
        // Speed").setNumber(rightFront.getSensorCollection().getPulseWidthVelocity());

        // leftFront.config_kP(0, SmartDashboard.getNumber("Left_P0", leftTalon_P0),
        // 100);
        // leftFront.config_kI(0, SmartDashboard.getNumber("Left_I0", leftTalon_I0),
        // 100);
        // leftFront.config_kD(0, SmartDashboard.getNumber("Left_D0", leftTalon_D0),
        // 100);
        // leftFront.config_kF(0, SmartDashboard.getNumber("Left_F0",
        // leftTalonFeedForward0), 100);
        // rightFront.config_kP(0, SmartDashboard.getNumber("Right_P0", rightTalon_P0),
        // 100);
        // rightFront.config_kI(0, SmartDashboard.getNumber("Right_I0", rightTalon_I0),
        // 100);
        // rightFront.config_kD(0, SmartDashboard.getNumber("Right_D0", rightTalon_D0),
        // 100);
        // rightFront.config_kF(0, SmartDashboard.getNumber("Right_F0",
        // rightTalonFeedForward0), 100);

        // leftFront.config_kP(1, SmartDashboard.getNumber("Left_P1", leftTalon_P1),
        // 100);
        // leftFront.config_kI(1, SmartDashboard.getNumber("Left_I1", leftTalon_I1),
        // 100);
        // leftFront.config_kD(1, SmartDashboard.getNumber("Left_D1", leftTalon_D1),
        // 100);
        // leftFront.config_kF(1, SmartDashboard.getNumber("Left_F1",
        // leftTalonFeedForward1), 100);
        // rightFront.config_kP(1, SmartDashboard.getNumber("Right_P1", rightTalon_P1),
        // 100);
        // rightFront.config_kI(1, SmartDashboard.getNumber("Right_I1", rightTalon_I1),
        // 100);
        // rightFront.config_kD(1, SmartDashboard.getNumber("Right_D1", rightTalon_D1),
        // 100);
        // rightFront.config_kF(1, SmartDashboard.getNumber("Right_F1",
        // rightTalonFeedForward1), 100);
    }

    @Override
    public void teleopInit() {
        climbMode = false;
        normalInit();

        modeSwitchButton.whenReleased(switchCommandsCommand);
    }

    private void normalInit() {
        System.out.println("NormalInit");
        // enable the light
        limeLight.setLEDMode(LEDMode.PipelineDefault);

        // start manual commands
        intakeCommand.start();
        teleopDrive.start();

        // setup button commands
        hatchButton.whenPressed(hatchDownCommand);
        hatchButton.whenReleased(hatchUpCommand);
        scoringMechanismButton.whenPressed(scoringMechanismControl);
        hatchStickButton.whenPressed(hatchStickControl);
        bumpLeftButton.whenPressed(bumpLeftCommand);
        bumpRightButton.whenPressed(bumpRightCommand);
        backupButton.whileHeld(backupCommand);
    }

    @Override
    public void autonomousInit() {
        teleopInit();
        backupLongButton.whileHeld(backupLongCommand);
    }

    @Override
    public void disabledInit() {
        // disable the light if the robot is disabled
        limeLight.setLEDMode(LEDMode.ForceOff);
        Scheduler.getInstance().removeAll();
    }

    @Override
    public void teleopPeriodic() {
        // run scheduler
        Scheduler.getInstance().run();

        if (!climbMode) {
            // set elevator position to 0 whenever the botton talon tach is hit
            Faults f = new Faults();
            elevator.getFaults(f);
            if (f.ReverseLimitSwitch) {
                elevator.setSelectedSensorPosition(0);
            }

            // start the different drive modes
            if (driverController.getTriggerAxis(cargoBallTrigger) > triggerThreshold) {
                ballDrive.start();
            } else if (driverController.getTriggerAxis(visionTargetTrigger) > triggerThreshold) {
                targetDrive.start();
            } else {
                if (!bumpLeftButton.get() && !bumpRightButton.get() && !backupButton.get() && !backupLongButton.get())
                    teleopDrive.start();
                limeLight.setPipeline(config.ids.pipelines.pipelineEmptyID);
            }

            // control elevator
            switch (operatorController.getPOV()) {
            case 0: // up
                position3.start();
                break;
            case 90: // right
                position2.start();
                break;
            case 180: // down
                home.start();
                break;
            case 270: // left
                position1.start();
                break;
            case -1: // none
                if (operatorController.getRawButton(5)) { // left bumper
                    position4.start();
                    break;
                } else if (operatorController.getRawButton(6)) { // right bumper
                    position5.start();
                    break;
                }
            default: // when none of the buttons are pressed
                if (OI.getElevatorSpeed() != 0)
                    manualElevatorCommand.start();
            }
        } else {
        }
    }

    @Override
    public void autonomousPeriodic() {
        teleopPeriodic();
    }

    private void setupTalon(WPI_TalonSRX talon, boolean left) {
        talon.configFactoryDefault(100);
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, pidLoopForward, 100);
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, pidLoopBackward, 100);
        talon.configNominalOutputForward(0, 100);
        talon.configNominalOutputReverse(0, 100);
        talon.configPeakOutputForward(1, 100);
        talon.configPeakOutputReverse(-1, 100);
        talon.selectProfileSlot(pidLoopForward, 0);
        talon.config_kF(pidLoopForward, left ? config.pids.forward.left.f : config.pids.forward.right.f, 100);
        talon.config_kP(pidLoopForward, left ? config.pids.forward.left.p : config.pids.forward.right.p, 100);
        talon.config_kI(pidLoopForward, left ? config.pids.forward.left.i : config.pids.forward.right.i, 100);
        talon.config_kD(pidLoopForward, left ? config.pids.forward.left.d : config.pids.forward.right.d, 100);
        talon.config_kF(pidLoopBackward, left ? config.pids.backward.left.f : config.pids.backward.right.f, 100);
        talon.config_kP(pidLoopBackward, left ? config.pids.backward.left.p : config.pids.backward.right.p, 100);
        talon.config_kI(pidLoopBackward, left ? config.pids.backward.left.i : config.pids.backward.right.i, 100);
        talon.config_kD(pidLoopBackward, left ? config.pids.backward.left.d : config.pids.backward.right.d, 100);
        talon.setSensorPhase(left ? config.pids.forward.left.sensorPhase : config.pids.forward.right.sensorPhase);
        talon.configMotionCruiseVelocity(config.pids.forward.left.cruiseVelocity);
        talon.configMotionAcceleration(config.pids.forward.left.acceleration);
        talon.configClosedloopRamp(0.125);

        // reset encoder
        talon.getSensorCollection().setPulseWidthPosition(0, 100);
    }

    public void toggleClimbMode() {
        DriverStation.reportWarning("Switched commands\n", false);
        climbMode = !climbMode;

        // reset commands
        clearCommands();
        modeSwitchButton.whenReleased(switchCommandsCommand);
        
        if (!climbMode) {
            normalInit();
        } else {
            manualClimbCommand.start();
            teleopDrive.start();
        }
    }

    private void clearCommands() {
        try { // very ugly hack to remove all buttons from the scheduler
              // in my opinion this should be part of the standard api
            Field buttons = Scheduler.class.getDeclaredField("m_buttons");
            buttons.setAccessible(true);
            buttons.set(Scheduler.getInstance(), null);

            Field m_firstCommand = Scheduler.class.getDeclaredField("m_firstCommand");
            m_firstCommand.setAccessible(true);
            var linkedListElementCl = Class.forName("edu.wpi.first.wpilibj.command.LinkedListElement");
            var getData = linkedListElementCl.getDeclaredMethod("getData");
            getData.setAccessible(true);
            var getNext = linkedListElementCl.getDeclaredMethod("getNext");
            getNext.setAccessible(true);

            // Loop through the commands
            Object element = m_firstCommand.get(Scheduler.getInstance());
            while (element != null) {
                Command command = (Command) getData.invoke(element);
                element = getNext.invoke(element);
                command.cancel();
            }
            // force canceling the commands / flush
            Scheduler.getInstance().run();

            // remove all the commands
            Scheduler.getInstance().removeAll();

            // tbh I'm not 100% sure why this works, but it does and that's all I need
        } catch (Throwable t) {
            throw new RuntimeException(t);
        }
    }
}

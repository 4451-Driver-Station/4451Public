package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public final class OI {
    //#region ids
    public static final int driverControllerID = 0;
    public static final int operatorControllerID = 1;

    /** Any value above this will trigger the triggers */
    public static final double triggerThreshold = 0.8;

    public static final XboxController.Hand visionTargetTrigger = XboxController.Hand.kLeft;
    public static final XboxController.Hand cargoBallTrigger = XboxController.Hand.kRight;
    public static final int hatchButtonID = 1;
    public static final int hatchStickButtonID = 2;
    public static final int backupButtonID = 3;
    public static final int backupLongButtonID = 4;
    public static final int scoringMechanismButtonID = 3;
    public static final int bumpLeftButtonID = 5;
    public static final int bumpRightButtonID = 6;
    public static final int modeSwitchButtonID = 8;
    public static final int resetButtonID = 8;
    public static final int downButtonID = 1;
    public static final int upRearButtonID = 2;
    public static final int upFrontButtonID = 3;
    //#endregion
    //#region objects
    public static final XboxController driverController = new XboxController(driverControllerID);
    public static final XboxController operatorController = new XboxController(operatorControllerID);
    public static final JoystickButton hatchButton = new JoystickButton(operatorController, hatchButtonID);
    public static final JoystickButton scoringMechanismButton = new JoystickButton(operatorController, scoringMechanismButtonID);
    public static final JoystickButton backupButton = new JoystickButton(driverController, backupButtonID);
    public static final JoystickButton backupLongButton = new JoystickButton(driverController, backupLongButtonID);
    public static final JoystickButton hatchStickButton = new JoystickButton(operatorController, hatchStickButtonID);
    public static final JoystickButton bumpLeftButton = new JoystickButton(driverController, bumpLeftButtonID);
    public static final JoystickButton bumpRightButton = new JoystickButton(driverController, bumpRightButtonID);
    public static final JoystickButton modeSwitchButton = new JoystickButton(driverController, modeSwitchButtonID);
    public static final JoystickButton resetButton = new JoystickButton(driverController, resetButtonID);
    public static final JoystickButton downButton = new JoystickButton(driverController, downButtonID);
    public static final JoystickButton upRearButton = new JoystickButton(driverController, upRearButtonID);
    public static final JoystickButton upFrontButton = new JoystickButton(driverController, upFrontButtonID);
    public static final JoystickButton switchCommandsButton = new JoystickButton(driverController, downButtonID);
    //#endregion
    //#region functions
    public static double getSpeed() {
        return -driverController.getY(Hand.kLeft);
    }
    public static double getRotation() {
        double raw = driverController.getX(Hand.kRight);
        return raw;
    }
    public static double getElevatorSpeed() {
        double raw = operatorController.getY(Hand.kLeft)/* / 1.5*/;
        return -raw;
    }
    public static double getClimbFrontSpeed() {
        double raw = operatorController.getY(Hand.kLeft);
        return -raw;
    }
    public static double getClimbRearSpeed() {
        double raw = operatorController.getY(Hand.kRight);
        return -raw;
    }
    //#endregion
}

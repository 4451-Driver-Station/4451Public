package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

import static frc.robot.OI.*;
import static frc.robot.RobotMap.*;

public class TeleopDrive extends Command {

    public TeleopDrive() {
        requires(driveTrain);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void execute() {
        double speed = getSpeed();
        double rotation = getRotation();

        if (elevator.getSelectedSensorPosition() > 400 || Robot.climbMode) {
            speed *= Math.sqrt(0.5);
            rotation *= Math.sqrt(0.5);
        }

        driveTrain.arcadeDrive(speed, rotation);
    
        if (Robot.climbMode) {
            topPlatformDrive.set(speed);
        } else {
            topPlatformDrive.set(0);
        }
    }

}
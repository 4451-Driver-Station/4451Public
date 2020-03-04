package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import static frc.robot.RobotMap.*;

public class BackupCommand extends Command {

    private final int setPoint;

    public BackupCommand(int setPoint) {
        this.setPoint = setPoint;
        requires(driveTrain);
    }

    @Override
    protected void initialize() {

        System.out.println("Resetting");
        driveTrain.resetEncoders();
        
        leftFront.selectProfileSlot(pidLoopBackward, 0);
        rightFront.selectProfileSlot(pidLoopBackward, 0);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void execute() {
        driveTrain.motionMagic(setPoint);
    }

} 
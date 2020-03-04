package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;

import static frc.robot.RobotMap.*;

public class ManualClimbCommand extends Command {

    public ManualClimbCommand() {
        requires(climb);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void execute() {
        double frontSpeed = OI.getClimbFrontSpeed() * 0.7;
        frontSpeed = Math.max(frontSpeed, -0.6);
        climb.frontClimb(frontSpeed);
        climb.rearClimb(OI.getClimbRearSpeed() * 0.8);
    }

}
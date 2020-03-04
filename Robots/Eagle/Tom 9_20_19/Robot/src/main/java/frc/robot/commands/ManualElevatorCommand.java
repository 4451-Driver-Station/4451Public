package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;

import static frc.robot.RobotMap.*;

public class ManualElevatorCommand extends Command {

    public ManualElevatorCommand() {
        requires(elevatorSystem);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void execute() {
        elevator.set(OI.getElevatorSpeed());
    }

}
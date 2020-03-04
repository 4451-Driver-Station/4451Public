package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import static frc.robot.RobotMap.*;

public class ElevatorCommand extends Command {

    private final double position;

    public ElevatorCommand(double position) {
        this.position = position;
        requires(elevatorSystem);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void execute() {
        elevatorSystem.setPosition(position);
    }

}
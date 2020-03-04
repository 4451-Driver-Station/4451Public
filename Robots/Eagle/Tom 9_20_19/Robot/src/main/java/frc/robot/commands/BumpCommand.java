package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import static frc.robot.RobotMap.*;

public class BumpCommand extends Command {

    private final boolean left;

    public BumpCommand(boolean left) {
        this.left = left;
        requires(driveTrain);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void execute() {
        double rotation = left ? -config.driveTrainConfig.bumpSpeed : config.driveTrainConfig.bumpSpeed;

        if (elevator.getSelectedSensorPosition() > 400) {
            rotation *= Math.sqrt(0.5);
        }

        driveTrain.arcadeDrive(0, rotation);
    }

}
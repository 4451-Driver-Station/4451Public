package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class SwitchCommandsCommand extends InstantCommand {

    @Override
    protected void initialize() {
        Robot.instance.toggleClimbMode();
    }

}
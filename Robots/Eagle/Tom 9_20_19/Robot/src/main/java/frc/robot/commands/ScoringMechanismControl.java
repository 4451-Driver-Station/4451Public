package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;

import static frc.robot.RobotMap.*;

public class ScoringMechanismControl extends InstantCommand {

    public ScoringMechanismControl() {
        requires(pneumatics);
    }

    @Override
    protected void execute() {
        pneumatics.setScoringMechanismState(!pneumatics.getScoringMechanismState());
    }
}
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;

import static frc.robot.RobotMap.*;

public class HatchCupControl extends InstantCommand  {

    private final boolean release;

    public HatchCupControl(boolean release) {
        this.release = release;
        requires(pneumatics);
    }

    @Override
    protected void execute() {
        pneumatics.setCups(release);
    }

}
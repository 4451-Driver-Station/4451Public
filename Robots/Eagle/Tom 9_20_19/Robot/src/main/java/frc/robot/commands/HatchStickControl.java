package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;

import static frc.robot.RobotMap.*;

public class HatchStickControl extends InstantCommand {

    public HatchStickControl() {
        requires(pneumatics);
    }

    @Override
    protected void execute() {
        pneumatics.setHatchStickState(!pneumatics.getHatchStickState());
    }
}
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import static frc.robot.RobotMap.*;

public class Intake extends Subsystem {

    @Override
    protected void initDefaultCommand() {
    }

    public void outward() {
        cargoIntake.set(-0.6);
    }

    public void inward(double speed) {
        cargoIntake.set(speed);
    }

    public void stop() {
        cargoIntake.set(0);
    }

}
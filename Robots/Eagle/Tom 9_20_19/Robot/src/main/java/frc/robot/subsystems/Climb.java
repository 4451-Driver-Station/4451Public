package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import static frc.robot.RobotMap.*;

public class Climb extends Subsystem {

    @Override
    protected void initDefaultCommand() {}

    public void frontClimb(double up) {
        topPlatformFront.set(up * 1.5);
    }

    public void rearClimb(double up) {
        topPlatformRear.set(-up);
    }

    public void drive(double speed) {
        topPlatformDrive.set(speed);
    }

}
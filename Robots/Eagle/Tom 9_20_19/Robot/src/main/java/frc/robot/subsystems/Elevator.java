package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import static frc.robot.RobotMap.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

public class Elevator extends Subsystem {

    @Override
    protected void initDefaultCommand() {
    }

    public void setPosition(double position) {
        elevator.set(ControlMode.MotionMagic, position, DemandType.AuxPID, 0);
    }

}
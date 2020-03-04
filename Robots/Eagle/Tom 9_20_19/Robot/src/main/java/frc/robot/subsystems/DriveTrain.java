package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.RobotMap.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

/** DriveTrain controls the drivetrain */
public class DriveTrain extends Subsystem {

    @Override
    protected void initDefaultCommand() {
    }

    public void arcadeDrive(double speed, double rotation) {
        // speed = Math.copySign(Math.min(Math.abs(speed), 1), speed);
        // rotation = Math.copySign(Math.min(Math.abs(rotation), 1), rotation);
        if (speed > 1)
            speed = 1;
        else if (speed < -1)
            speed = -1;
        if (rotation > 1)
            rotation = 1;
        else if (rotation < -1)
            rotation = -1;

        if (Math.abs(speed) < config.driveTrainConfig.deadband)
            speed = 0;

        if (Math.abs(rotation) < config.driveTrainConfig.deadband)
            rotation = 0;

        speed = Math.copySign(speed * speed, speed);
        rotation = Math.copySign(rotation * rotation, rotation);

        double speedLeft;
        double speedRight;
        double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(rotation)), speed);
        if ((speed >= 0.0) == (rotation >= 0.0)) {
            speedLeft = maxInput;
            speedRight = speed - rotation;
        } else {
            speedLeft = speed + rotation;
            speedRight = maxInput;
        }

        // select different profiles based on the direction
        leftFront.selectProfileSlot(speedLeft >= 0 ? pidLoopForward : pidLoopBackward, 0);
        rightFront.selectProfileSlot(speedRight >= 0 ? pidLoopForward : pidLoopBackward, 0);

        leftFront.set(ControlMode.Velocity, speedLeft * config.driveTrainConfig.maxRPM * 4096D / 600D);
        rightFront.set(ControlMode.Velocity, speedRight * config.driveTrainConfig.maxRPM * 4096D / 600D);
        // leftFront.set(speedLeft);
        // rightFront.set(speedRight);
    }

    public void resetEncoders() {
        leftFront.setSelectedSensorPosition(0);
        rightFront.setSelectedSensorPosition(0);
    }

    public void motionMagic(int point) {
        leftFront.selectProfileSlot(0, pidLoopBackward);
        rightFront.selectProfileSlot(0, pidLoopBackward);
        if (leftFront.getClosedLoopError() <= 0)
            leftFront.set(ControlMode.MotionMagic, point, DemandType.Neutral, 0);
        if (rightFront.getClosedLoopError() <= 0)
            rightFront.set(ControlMode.MotionMagic, point, DemandType.Neutral, 0);

        SmartDashboard.putNumber("Error left", leftFront.getClosedLoopError());
        SmartDashboard.putNumber("Error right", rightFront.getClosedLoopError());
    }

}

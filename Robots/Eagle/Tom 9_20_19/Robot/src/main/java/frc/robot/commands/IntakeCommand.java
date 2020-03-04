package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import static frc.robot.RobotMap.*;
import static frc.robot.OI.*;

public class IntakeCommand extends Command {

    public IntakeCommand() {
        requires(intake);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    private boolean wasRunning = false;
    private boolean startedWithBall = false;
    private boolean hasSeenBallSinceWeStarted = false;

    @Override
    protected void execute() {
        if (operatorController.getTriggerAxis(Hand.kRight) > triggerThreshold) {
            if (!wasRunning) {
                wasRunning = true;
                startedWithBall = !ballLimitSwitch.get();
                hasSeenBallSinceWeStarted = false;
            }
            System.out.println(hasSeenBallSinceWeStarted);

            if (!startedWithBall && (!ballLimitSwitch.get() || hasSeenBallSinceWeStarted)) {
                intake.stop();
                hasSeenBallSinceWeStarted = true;
            }else{
                intake.inward(startedWithBall ? 1 : 0.6);
            }
        } else if (operatorController.getTriggerAxis(Hand.kLeft) > triggerThreshold) {
            intake.outward();
        } else {
            intake.stop();
            wasRunning = false;
        }
    }

}
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import static frc.robot.RobotMap.*;

public class Pneumatics extends Subsystem {

    @Override
    protected void initDefaultCommand() {
    }

    public void setCups(boolean release) {
        hatchCup1.set(release != config.pneumaticInvertations.hatchCup1);
        hatchCup2.set(release != config.pneumaticInvertations.hatchCup2);
    }

    public boolean getScoringMechanismState() {
        return scoringMechanism.get() != config.pneumaticInvertations.scoringMechanism;
    }

    public void setScoringMechanismState(boolean front) {
        scoringMechanism.set(front != config.pneumaticInvertations.scoringMechanism);
    }

    public boolean getHatchStickState() {
        return hatchStick.get() != config.pneumaticInvertations.hatchStick;
    }

    public void setHatchStickState(boolean down) {
        hatchStick.set(down != config.pneumaticInvertations.hatchStick);
    }

    public void defaultPosition() {
        setHatchStickState(false);
        setScoringMechanismState(false);
    }

}
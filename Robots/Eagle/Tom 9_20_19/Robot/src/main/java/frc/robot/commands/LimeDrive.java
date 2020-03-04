package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import static frc.robot.RobotMap.*;
import java.util.ArrayList;
import java.util.List;

import frc.robot.OI;
import frc.robot.sensors.LimeLight.CamMode;
import frc.robot.sensors.LimeLight.LEDMode;

public class LimeDrive extends Command {

    private List<Double> angles = new ArrayList<>();

    private final Mode mode;

    public static enum Mode {
        Ball(0.75), Hatch(1);

        private final double speed;

        Mode(double speed) {
            this.speed = speed;
        }
    }

    public LimeDrive(Mode mode) {
        this.mode = mode;
        requires(limeLight);
        requires(driveTrain);
        requires(lidarLeft);
        requires(lidarRight);
    }

    @Override
    protected void initialize() {
        angles.clear();

        limeLight.setCamMode(CamMode.VisionProcessor);
        limeLight.setLEDMode(LEDMode.PipelineDefault);

        switch (mode) {
        case Hatch:
            pneumatics.setScoringMechanismState(false);
            limeLight.setPipeline(config.ids.pipelines.pipelineTargetID);
            break;
        case Ball:
            pneumatics.setHatchStickState(false);
            limeLight.setPipeline(config.ids.pipelines.pipelineBallID);
            pneumatics.setScoringMechanismState(true);
            pneumatics.setHatchStickState(true);
            break;
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void execute() {
        double speed = 0;
        double direction = 0;

        double distanceLeft = lidarLeft.getInches() + config.limeDrive.leftOffset;
        double distanceRight = lidarRight.getInches() + config.limeDrive.rightOffset;
        double distance = Math.min(distanceLeft, distanceRight) / 100D;


        distance = Math.max(distance, 0);
        distance = Math.min(distance, 1);

        if (limeLight.hasTarget()) {
            // no need for tolerance: if abs(direction) is lower than 0.2, it is discarded
            direction = Math.copySign(
                    Math.pow(Math.abs((double) limeLight.getX()) / config.limeDrive.turnSpeed, 3D / 5D),
                    limeLight.getX());

            direction = Math.copySign(Math.abs(direction) + 0.1, direction);

            speed = Math.sqrt(distance) / 1.4;
        }

        speed *= Math.sqrt(mode.speed);
        speed = OI.getSpeed();
        
        if (Math.abs(OI.getRotation()) > 0.2) 
            direction = OI.getRotation();
        driveTrain.arcadeDrive(speed, direction);

        switch (mode) {
        case Hatch:
            break;
        case Ball:
            // if we don't have the ball yet: activate intake, push the scoring mechanism
            // forward, put the stick down
            if (ballLimitSwitch.get())
                intake.inward(0.6);
            break;
        }
    }
}
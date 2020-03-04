package frc.robot.sensors;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import static frc.robot.RobotMap.*;

/** Lidar measures the distance to the nearest object */
public class Lidar extends Subsystem {

    private final Counter counter;
    private double lastMeasurement;

    public Lidar(int channel) {
        counter = new Counter(new DigitalInput(channel));
        counter.setMaxPeriod(1);
        counter.setSemiPeriodMode(true);
        counter.reset();
    }

    protected void initDefaultCommand() {
    }

    protected double getRaw() { // the raw value is actually in centimeters
        return counter.getPeriod() * 100000D;
    }

    /**
     * @return the distance in inches
     */
    public double getInches() {
        if (counter.get() < 1)
            return lastMeasurement;
        lastMeasurement = getRaw() / 2.547D; // convert to inches
        if (lastMeasurement < config.lidar.minOffset) // out of range check
            return 100D;
        return lastMeasurement - config.lidar.offset;
    }

}
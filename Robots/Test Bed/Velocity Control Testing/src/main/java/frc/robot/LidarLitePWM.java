package frc.robot;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalSource;

public class LidarLitePWM {
	public static final double ENCODER_CALIBRATION_OFFSET = 0.0;
	private Counter counter;

	public LidarLitePWM(DigitalSource source) {
		counter = new Counter(source);

		counter.setMaxPeriod(1);
		counter.setSemiPeriodMode(true);
		counter.reset();
	}

	public double getDistance() {
		double cm;

		if(counter.get()<1) {
			System.out.println("Waiting for Lidar distance measurement");
			return 0;
		}
		cm = (counter.getPeriod() * 1000000.0 / 10.0) + ENCODER_CALIBRATION_OFFSET;
		return cm;

	}

	public double getCounter() {
		return counter.get();
	}
}

package frc.robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.sensors.LimeLight;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;

public final class RobotMap {

    public final static RobotMap config;

    private static String getMode() {
        try (BufferedReader reader = new BufferedReader(new FileReader(new File("/etc/mode")))) {
            return reader.readLine();
        } catch (Throwable t) {
            throw new RuntimeException(t);
        }
    }

    static {
        ObjectMapper mapper = new ObjectMapper(new YAMLFactory());
        try {
            config = mapper.readValue(new File(Filesystem.getDeployDirectory(), getMode()+".yaml"), RobotMap.class);
            config.claimMotors();
        } catch (Throwable t) {
            throw new RuntimeException(t);
        }
    }

    public boolean PRODUCTION = false;

    public static class IDs {
        public static class MotorIDs {
            public int leftFront;
            public int rightFront;
            public int leftMiddle;
            public int rightMiddle;
            public int leftRear;
            public int rightRear;
            public int elevator;
            public int cargoIntake;
            public int topPlatformFront;
            public int topPlatformRear;
            public int topPlatformDrive;
        }

        public static class DIOIDs {
            public int lidarLeft;
            public int lidarRight;
            public int lidarFront;
            public int lidarRear;
            public int ballLimitSwitch;
        }

        public static class PneumaticIDs {
            public int hatchStick;
            public int scoringMechanism;
            public int hatchCup1;
            public int hatchCup2;
        }

        public static class PipelineIDs {
            /** Detects the target */
            public int pipelineTargetID;
            /** Detects the ball */
            public int pipelineBallID;
            /** Detects nothing and turns the light off */
            public int pipelineEmptyID;
        }

        public MotorIDs motors;
        public DIOIDs dio;
        public PneumaticIDs pneumatics;
        public PipelineIDs pipelines;
    }

    public static class PID {

        @JsonCreator
        public PID(@JsonProperty("maxSpeed") double maxSpeed, @JsonProperty("p") double p, @JsonProperty("i") double i,
                @JsonProperty("d") double d, @JsonProperty("cruiseVelocity") int cruiseVelocity,
                @JsonProperty("acceleration") int acceleration, @JsonProperty("sensorPhase") boolean sensorPhase) {
            this.maxSpeed = maxSpeed;
            this.f = 1023 / maxSpeed;
            this.p = p;
            this.i = i;
            this.d = d;
            this.cruiseVelocity = cruiseVelocity;
            this.acceleration = acceleration;
            this.sensorPhase = sensorPhase;
        }

        /** The theoretical max speed is used to calculate the feedforward */
        public double maxSpeed;
        /** F value for the PID */
        public double f;
        /** P value for the PID */
        public double p;
        /** I value for the PID */
        public double i;
        /** D value for the PID */
        public double d;
        /** max speed */
        public int cruiseVelocity;
        /** max acceleration */
        public int acceleration;
        /** sensor phase of the encoder */
        public boolean sensorPhase;
    }

    public static class DriveTrain {
        public PID left;
        public PID right;
    }

    public static class PIDs {
        public PID elevator;
        public DriveTrain forward;
        public DriveTrain backward;
    }

    public static class PneumaticInvertations {
        public boolean hatchStick;
        public boolean scoringMechanism;
        public boolean hatchCup1;
        public boolean hatchCup2;
    }
    public static class DriveTrainConfig {
        public double maxRPM;
        public double deadband;
        public double bumpSpeed;
    }
    public static class LidarConfig {
        public double minOffset;
        public double offset;
    }
    public static class LimeDrive {
        public double leftOffset;
        public double rightOffset;
        /** Determines how aggressivly the limedrive turns */
        public double turnSpeed;
    }

    public IDs ids;
    public PIDs pids;
    public PneumaticInvertations pneumaticInvertations;
    @JsonProperty("driveTrain")
    public DriveTrainConfig driveTrainConfig;
    public LidarConfig lidar;
    public LimeDrive limeDrive;

    // #region tuning constants

    /** The pid loop id for driving forwards */
    public static final int pidLoopForward = 0;
    /** The pid loop id for driving backwards */
    public static final int pidLoopBackward = 1;

    // #endregion
    // #region sensor subsystems
    public static LimeLight limeLight = new LimeLight();
    public static frc.robot.sensors.Lidar lidarLeft;
    public static frc.robot.sensors.Lidar lidarRight;
    public static frc.robot.sensors.Lidar lidarFront;
    public static frc.robot.sensors.Lidar lidarBack;
    public static DigitalInput ballLimitSwitch;
    // #endregion
    // #region motor objects & related subsystems
    public static WPI_TalonSRX leftFront;
    public static WPI_TalonSRX rightFront;
    public static WPI_VictorSPX leftMiddle;
    public static WPI_VictorSPX rightMiddle;
    public static WPI_VictorSPX leftRear;
    public static WPI_VictorSPX rightRear;
    public static WPI_TalonSRX elevator;
    public static WPI_VictorSPX cargoIntake;
    public static WPI_TalonSRX topPlatformFront;
    public static WPI_TalonSRX topPlatformRear;
    public static WPI_VictorSPX topPlatformDrive;
    public static PowerDistributionPanel pdp;

    public static frc.robot.subsystems.DriveTrain driveTrain;
    public static Elevator elevatorSystem;
    public static Intake intake;
    public static Climb climb;
    // #endregion
    // #region pneumatics & related subsystems
    public static Solenoid hatchCup1;
    public static Solenoid hatchCup2;
    public static Solenoid scoringMechanism;
    public static Solenoid hatchStick;
    public static Pneumatics pneumatics;
    // #endregion

    private void claimMotors() {
        leftFront = new WPI_TalonSRX(ids.motors.leftFront);
        rightFront = new WPI_TalonSRX(ids.motors.rightFront);
        leftMiddle = new WPI_VictorSPX(ids.motors.leftMiddle);
        rightMiddle = new WPI_VictorSPX(ids.motors.rightMiddle);
        leftRear = new WPI_VictorSPX(ids.motors.leftRear);
        rightRear = new WPI_VictorSPX(ids.motors.rightRear);
        elevator = new WPI_TalonSRX(ids.motors.elevator);
        cargoIntake = new WPI_VictorSPX(ids.motors.cargoIntake);
        topPlatformFront = new WPI_TalonSRX(ids.motors.topPlatformFront);
        topPlatformRear = new WPI_TalonSRX(ids.motors.topPlatformRear);
        topPlatformDrive = new WPI_VictorSPX(ids.motors.topPlatformDrive);
        pdp = new PowerDistributionPanel(0);

        driveTrain = new frc.robot.subsystems.DriveTrain();
        elevatorSystem = new Elevator();
        intake = new Intake();
        climb = new Climb();

        hatchCup1 = new Solenoid(ids.pneumatics.hatchCup1);
        hatchCup2 = new Solenoid(ids.pneumatics.hatchCup2);
        scoringMechanism = new Solenoid(ids.pneumatics.scoringMechanism);
        hatchStick = new Solenoid(ids.pneumatics.hatchStick);
        pneumatics = new Pneumatics();

        lidarLeft = new frc.robot.sensors.Lidar(config.ids.dio.lidarLeft);
        lidarRight = new frc.robot.sensors.Lidar(config.ids.dio.lidarRight);
        lidarFront = new frc.robot.sensors.Lidar(config.ids.dio.lidarFront);
        lidarBack = new frc.robot.sensors.Lidar(config.ids.dio.lidarRear);
        ballLimitSwitch = new DigitalInput(config.ids.dio.ballLimitSwitch);
    }
}

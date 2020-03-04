package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;

/** Helper class for the LimeLight */
public class LimeLight extends Subsystem {

    private final static String tableName = "limelight";

    public static enum LEDMode {
        /** use the LED Mode set in the current pipeline */
        PipelineDefault, ForceOff, ForceBlink, ForceOn
    }

    public static enum CamMode {
        VisionProcessor,
        /** increases exposure, disables vision processing */
        DriverCamera
    }

    @Override
    protected void initDefaultCommand() {
    }

    /** Whether the limelight has any valid targets */
    public boolean hasTarget()  {
        return get("tv") == 1;
    }

    /** Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees) */
    public double getX()  {
        return get("tx");
    }

    /** Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees) */
    public double getY() {
        return get("ty");
    }

    /** Target Area (0% of image to 100% of image) */
    public double getArea() {
        return get("ta");
    }

    /** Skew or rotation (-90 degrees to 0 degrees) */
    public double getSkew() {
        return get("ts");
    }

    /**
     * The pipeline’s latency contribution (ms) Add at least 11ms for image capture
     * latency.
     */
    public double getLatency() {
        return get("tl");
    }

    /**
     * @param contour one of the three contours (0..2)
     * @return Raw Screenspace X
     */
    public double getRawScreenSpaceX(int contour) {
        if (contour < 0 || contour > 2)
            throw new IllegalArgumentException("contour must be between 0 and 2");
        return get("tx" + contour);
    }

    /**
     * @param contour one of the three contours (0..2)
     * @return Raw Screenspace Y
     */
    public double getRawScreenSpaceY(int contour) {
        if (contour < 0 || contour > 2)
            throw new IllegalArgumentException("contour must be between 0 and 2");
        return get("ty" + contour);
    }

    /**
     * @param contour one of the three contours (0..2)
     * @return Area (0% of image to 100% of image)
     */
    public double getRawArea(int contour) {
        if (contour < 0 || contour > 2)
            throw new IllegalArgumentException("contour must be between 0 and 2");
        return get("ta" + contour);
    }

    /**
     * @param contour one of the three contours (0..2)
     * @return Skew or rotation (-90 degrees to 0 degrees)
     */
    public double getRawSkew(int contour) {
        if (contour < 0 || contour > 2)
            throw new IllegalArgumentException("contour must be between 0 and 2");
        return get("ts" + contour);
    }

    /**
     * @param crosshair one of the two crosshairs (0..1)
     * @return Crosshair X in normalized screen space
     */
    public double getRawCrossHairX(int crosshair) {
        if (crosshair < 0 || crosshair > 1)
            throw new IllegalArgumentException("crosshair must be between 0 and 1");
        return get("cx" + crosshair);
    }

    /**
     * @param crosshair one of the two crosshairs (0..1)
     * @return Crosshair Y in normalized screen space
     */
    public double getRawCrossHairY(int crosshair) {
        if (crosshair < 0 || crosshair > 1)
            throw new IllegalArgumentException("crosshair must be between 0 and 1");
        return get("cy" + crosshair);
    }

    /** Gets limelight’s LED state */
    public LEDMode getLEDMode() {
        return LEDMode.values()[(int) get("ledMode")];
    }

    /** Sets limelight’s LED state */
    public void setLEDMode(LEDMode mode) {
        set("ledMode", mode.ordinal());
    }

    /**
     * Gets limelight’s operation mode
     */
    public CamMode getCamMode() {
        return CamMode.values()[(int) get("camMode")];
    }

    /**
     * Sets limelight’s operation mode
     * 
     * @param mode the mode
     */
    public void setCamMode(CamMode mode) {
        set("camMode", mode.ordinal());
    }

    /**
     * Sets limelight’s current pipeline
     * 
     * @param pipeline Select pipeline 0..9
     */
    public void setPipeline(int pipeline) {
        if (pipeline < 0 || pipeline > 9)
            throw new IllegalArgumentException("pipeline must be between 0 and 9");
        set("pipeline", pipeline);
    }

    /**
     * Allows users to take snapshots during a match. If true takes two snapshots
     * per second
     */
    public boolean getSnapshotMode() {
        return get("snapshot") == 1;
    }

    /**
     * Allows users to take snapshots during a match. If true takes two snapshots
     * per second
     */
    public void setSnapshotMode(boolean enabled) {
        set("snapshot", enabled ? 1 : 0);
    }

    protected double get(String name) {
        return NetworkTableInstance.getDefault().getTable(tableName).getEntry(name).getDouble(0);
    }

    protected void set(String name, double value) {
        NetworkTableInstance.getDefault().getTable(tableName).getEntry(name).setDouble(value);
    }
}
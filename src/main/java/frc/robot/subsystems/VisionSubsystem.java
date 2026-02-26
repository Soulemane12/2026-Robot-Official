package frc.robot.subsystems;

import java.util.function.BiConsumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/**
 * VisionSubsystem handles all Limelight camera operations for AprilTag tracking.
 *
 * IMPORTANT: Change LIMELIGHT_NAME to match your Limelight's network name!
 * Common names: "limelight", "limelight-front", "limelight-back", etc.
 *
 * To find your Limelight name:
 * 1. Go to http://limelight.local:5801 (or your Limelight's IP)
 * 2. Check the Settings tab for the hostname
 * 3. Or check NetworkTables in Shuffleboard to see available tables
 */
public class VisionSubsystem extends SubsystemBase {

    // CHANGE THIS TO YOUR LIMELIGHT NAME!
    // Common names: "limelight", "limelight-front", "limelight-back"
    // Check your Limelight web interface at http://limelight.local:5801
    private static final String LIMELIGHT_NAME = "limelight-front";

    // Launch velocity constant (meters per second)
    private static final double LAUNCH_VELOCITY = 5.2;

    private final BiConsumer<Pose2d, Double> visionUpdater;

    /**
     * Creates a VisionSubsystem without vision measurement updates.
     * Use this constructor if you only need basic vision data without odometry fusion.
     */
    public VisionSubsystem() {
        this(null);
    }

    /**
     * Creates a VisionSubsystem with vision measurement updates to the drivetrain.
     *
     * @param visionUpdater Callback to update drivetrain with vision pose estimates.
     *                      Accepts (Pose2d pose, Double timestamp)
     */
    public VisionSubsystem(BiConsumer<Pose2d, Double> visionUpdater) {
        this.visionUpdater = visionUpdater;
    }

    @Override
    public void periodic() {
        // Update drivetrain with vision measurements if callback provided
        addLatestEstimate();

        SmartDashboard.putBoolean("Vision/HasTarget", hasValidTarget());
        SmartDashboard.putNumber("Vision/Distance_m", getDistanceToAprilTag());
    }

    /**
     * Gets the horizontal offset from crosshair to target (degrees).
     * Positive = target is to the right of crosshair
     * Negative = target is to the left of crosshair
     */
    public double getTargetTX() {
        return NetworkTableInstance.getDefault()
            .getTable(LIMELIGHT_NAME)
            .getEntry("tx")
            .getDouble(0.0);
    }

    /**
     * Gets the vertical offset from crosshair to target (degrees).
     * This is often used as a proxy for distance - larger ty = closer
     */
    public double getTargetTY() {
        return NetworkTableInstance.getDefault()
            .getTable(LIMELIGHT_NAME)
            .getEntry("ty")
            .getDouble(0.0);
    }

    /**
     * Gets the target area (0% to 100% of image).
     * Larger area = closer to the target.
     * This can be used for distance control.
     */
    public double getTargetArea() {
        return NetworkTableInstance.getDefault()
            .getTable(LIMELIGHT_NAME)
            .getEntry("ta")
            .getDouble(0.0);
    }

    /**
     * Gets the horizontal angle offset (same as TX).
     * Used for rotation correction to face the target.
     */
    public double getTargetAngle() {
        return getTargetTX();
    }

    /**
     * Checks if the Limelight has a valid target.
     * @return true if a valid target is detected
     */
    public boolean hasValidTarget() {
        return NetworkTableInstance.getDefault()
            .getTable(LIMELIGHT_NAME)
            .getEntry("tv")
            .getDouble(0.0) == 1.0;
    }

    /**
     * Gets the ID of the currently tracked AprilTag.
     * @return The tag ID, or -1 if no tag is detected
     */
    public double getTagID() {
        return NetworkTableInstance.getDefault()
            .getTable(LIMELIGHT_NAME)
            .getEntry("tid")
            .getDouble(-1);
    }

    /**
     * Gets the 3D camera transform data.
     * Returns [x, y, z, roll, pitch, yaw] relative to the target
     */
    public double[] get3DPose() {
        return NetworkTableInstance.getDefault()
            .getTable(LIMELIGHT_NAME)
            .getEntry("camtran")
            .getDoubleArray(new double[6]);
    }

    /**
     * Gets the distance to the target using the target area.
     * Note: This is an approximation. For more accurate distance,
     * you may need to calibrate based on your specific tag size.
     *
     * @return Approximate distance (higher area = closer)
     */
    public double getDistanceFromArea() {
        double area = getTargetArea();
        if (area <= 0) return -1;
        // This is a rough approximation - you may need to tune this
        // based on your AprilTag size and camera setup
        return 1.0 / Math.sqrt(area / 100.0);
    }

    /**
     * Sets the pipeline index on the Limelight.
     * @param pipelineIndex The pipeline to switch to (0-9)
     */
    public void setPipeline(int pipelineIndex) {
        NetworkTableInstance.getDefault()
            .getTable(LIMELIGHT_NAME)
            .getEntry("pipeline")
            .setNumber(pipelineIndex);
    }

    /**
     * Gets the current pipeline index.
     * @return The active pipeline index (0-9)
     */
    public int getPipeline() {
        return (int) NetworkTableInstance.getDefault()
            .getTable(LIMELIGHT_NAME)
            .getEntry("getpipe")
            .getDouble(0.0);
    }

    /**
     * Sets the LED mode.
     * @param mode 0=pipeline default, 1=off, 2=blink, 3=on
     */
    public void setLEDMode(int mode) {
        NetworkTableInstance.getDefault()
            .getTable(LIMELIGHT_NAME)
            .getEntry("ledMode")
            .setNumber(mode);
    }

    /**
     * Gets the current LED mode.
     * @return The active LED mode (0-3)
     */
    public int getLEDMode() {
        return (int) NetworkTableInstance.getDefault()
            .getTable(LIMELIGHT_NAME)
            .getEntry("ledMode")
            .getDouble(0.0);
    }

    /**
     * Gets the name of the Limelight being used.
     * Useful for debugging.
     */
    public String getLimelightName() {
        return LIMELIGHT_NAME;
    }

    /**
     * Gets the distance to the AprilTag in meters using the 3D pose data.
     * The distance is calculated as the Euclidean distance (sqrt(x^2 + y^2 + z^2)).
     *
     * @return Distance to AprilTag in meters, or 0.0 if no valid target
     */
    public double getDistanceToAprilTag() {
        if (!hasValidTarget()) {
            return 0.0;
        }

        // Try getting distance from targetpose_cameraspace first (more reliable)
        double[] targetPose = NetworkTableInstance.getDefault()
            .getTable(LIMELIGHT_NAME)
            .getEntry("targetpose_cameraspace")
            .getDoubleArray(new double[6]);

        if (targetPose.length >= 3) {
            double x = targetPose[0];
            double y = targetPose[1];
            double z = targetPose[2];

            // Check if we have valid non-zero data
            if (x != 0.0 || y != 0.0 || z != 0.0) {
                return Math.sqrt(x * x + y * y + z * z);
            }
        }

        // Fallback to camtran if targetpose_cameraspace doesn't work
        double[] pose3d = get3DPose();

        // Check if we have valid pose data
        if (pose3d.length < 3) {
            return 0.0;
        }

        // Calculate 3D distance: sqrt(x^2 + y^2 + z^2)
        double x = pose3d[0];
        double y = pose3d[1];
        double z = pose3d[2];

        // Check if we have valid non-zero data
        if (x == 0.0 && y == 0.0 && z == 0.0) {
            return 0.0;
        }

        return Math.sqrt(x * x + y * y + z * z);
    }

    /**
     * Calculates the shooter angle needed to hit the target.
     * Uses the projectile motion formula: angle = (arcsin(R*g / v₀²)) / 2
     * where:
     *   R = distance to AprilTag (meters)
     *   g = 9.81 m/s² (gravity)
     *   v₀ = 5.2 m/s (launch velocity)
     *
     * @return Shooter angle in degrees, or 0.0 if no valid target or calculation error
     */
    public double getShooterAngle() {
        double R = getDistanceToAprilTag();

        if (R <= 0.0) {
            return 0.0;
        }

        double g = 9.81; // m/s² (gravity)
        double v0 = 5.2; // m/s (launch velocity)

        // Calculate the ratio for arcsin: (R*g) / (v₀²)
        double ratio = (R * g) / (v0 * v0);

        // Check if ratio is valid for arcsin (must be between -1 and 1)
        if (ratio > 1.0 || ratio < -1.0) {
            // Distance is too far for the given velocity
            return 0.0;
        }

        // Calculate angle: (arcsin(ratio)) / 2, then convert to degrees
        double angleRadians = Math.asin(ratio) / 2.0;
        double angleDegrees = Math.toDegrees(angleRadians);

        return angleDegrees;
    }

    /**
     * Calculates the launch angle needed to hit the target.
     * Uses the formula: angle = arcsin(distance / velocity)
     * where velocity = 5.2 m/s
     *
     * @return Launch angle in degrees, or 0.0 if no valid target or calculation error
     * @deprecated Use getShooterAngle() instead for projectile motion calculation
     */
    @Deprecated
    public double getLaunchAngle() {
        double distance = getDistanceToAprilTag();

        if (distance <= 0.0) {
            return 0.0;
        }

        // Calculate the ratio for arcsin
        double ratio = distance / LAUNCH_VELOCITY;

        // Check if ratio is valid for arcsin (must be between -1 and 1)
        if (ratio > 1.0 || ratio < -1.0) {
            // Distance is too far for the given velocity
            return 0.0;
        }

        // Calculate angle in radians then convert to degrees
        double angleRadians = Math.asin(ratio);
        double angleDegrees = Math.toDegrees(angleRadians);

        return angleDegrees;
    }

    /**
     * Gets the latest MegaTag1 pose estimate from the Limelight.
     * Returns null if no estimate is available.
     *
     * @return PoseEstimate with robot pose, timestamp, and tag info
     */
    public PoseEstimate getPoseMT1() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);
    }

    /**
     * Determines if a pose estimate should be accepted and used for odometry updates.
     * Can be enhanced with more sophisticated filtering based on tag count, distance, etc.
     *
     * @param estimate The pose estimate to validate
     * @return true if the estimate should be accepted
     */
    public boolean shouldAcceptEstimate(PoseEstimate estimate) {
        // Basic validation: check if estimate exists and has valid data
        if (estimate == null || estimate.pose == null) {
            return false;
        }

        // Check if we have at least one tag
        if (estimate.tagCount < 1) {
            return false;
        }

        // Future enhancements could include:
        // - Distance filtering (reject if tags too far)
        // - Area filtering (reject if tags too small)
        // - Ambiguity filtering
        // - Velocity consistency checks

        return true;
    }

    /**
     * Adds the latest vision pose estimate to the drivetrain's Kalman filter.
     * Called automatically in periodic() if a vision updater callback was provided.
     */
    public void addLatestEstimate() {
        // Skip if no callback provided
        if (visionUpdater == null) {
            return;
        }

        double currentTimeSeconds = Timer.getFPGATimestamp();
        PoseEstimate latestEstimate = getPoseMT1();

        if (shouldAcceptEstimate(latestEstimate)) {
            // Calculate timestamp: current time minus latency
            double estimateTimeSeconds = currentTimeSeconds - latestEstimate.timestampSeconds;

            // Call the drivetrain's addVisionMeasurement
            visionUpdater.accept(latestEstimate.pose, estimateTimeSeconds);
        }
    }
}

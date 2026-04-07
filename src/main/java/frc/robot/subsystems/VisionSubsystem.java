package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class VisionSubsystem extends SubsystemBase {

    private static final String LIMELIGHT = Constants.VisionConstants.LIMELIGHT_TURRET;

    // 2026 REBUILT field boundaries in meters (blue origin)
    private static final double FIELD_MAX_X = 16.54;
    private static final double FIELD_MAX_Y = 8.02;

    // Reject vision updates during fast rotation (degrees/sec)
    private static final double MAX_OMEGA_DEG_PER_SEC = 360.0;

    private final CommandSwerveDrivetrain m_drivetrain;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        var state = m_drivetrain.getState();
        double yawDeg    = state.Pose.getRotation().getDegrees();
        double omegaDeg  = Math.toDegrees(state.Speeds.omegaRadiansPerSecond);
        boolean spinning = Math.abs(omegaDeg) > MAX_OMEGA_DEG_PER_SEC;

        // Feed heading to the single turret Limelight every frame (required by MegaTag2)
        LimelightHelpers.SetRobotOrientation(LIMELIGHT, yawDeg, 0, 0, 0, 0, 0);

        // LL4 IMU mode: seed from external heading while disabled, blend when enabled
        // Requires LimelightHelpers v1.14. If your LimelightHelpers doesn't have SetIMUMode,
        // update to v1.14 from docs.limelightvision.io/docs/resources/downloads
        setIMUMode(LIMELIGHT, DriverStation.isDisabled() ? 1 : 4);

        if (!spinning) {
            processEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT), "Turret");
        }

        SmartDashboard.putBoolean("Vision/Turret/HasTarget", hasValidTarget());
        SmartDashboard.putBoolean("Vision/Spinning",        spinning);
    }

    private void processEstimate(PoseEstimate est, String label) {
        // Null check — getBotPoseEstimate_wpiBlue_MegaTag2 can return null
        if (est == null) return;
        if (est.tagCount < 1) return;
        if (est.avgTagDist > Constants.VisionConstants.MAX_TAG_DIST_M) return;

        // Field boundary check — reject poses outside the 2026 field perimeter
        double px = est.pose.getX(), py = est.pose.getY();
        if (px < 0 || px > FIELD_MAX_X || py < 0 || py > FIELD_MAX_Y) return;

        // Scale stdDevs by distance AND tag count:
        // Multi-tag sightings are more constrained, so trust them more
        double baseStdDev = (est.tagCount >= 2)
                ? Constants.VisionConstants.BASE_STD_DEV * 0.5
                : Constants.VisionConstants.BASE_STD_DEV;
        double slopeStdDev = (est.tagCount >= 2)
                ? Constants.VisionConstants.STD_DEV_PER_METER * 0.6
                : Constants.VisionConstants.STD_DEV_PER_METER;

        double stdDev = baseStdDev + slopeStdDev * est.avgTagDist;

        m_drivetrain.addVisionMeasurement(
            est.pose,
            est.timestampSeconds,
            VecBuilder.fill(stdDev, stdDev, 9999999)
        );

        SmartDashboard.putNumber("Vision/" + label + "/StdDev",     stdDev);
        SmartDashboard.putNumber("Vision/" + label + "/TagCount",   est.tagCount);
        SmartDashboard.putNumber("Vision/" + label + "/AvgTagDist", est.avgTagDist);
    }

    // Writes IMU mode to Limelight via NT. Mode 1 = seed from external, 4 = internal+assist.
    // Native to LimelightHelpers v1.14; calling directly via NT for compatibility.
    private static void setIMUMode(String llName, int mode) {
        NetworkTableInstance.getDefault()
            .getTable(llName)
            .getEntry("imumode_set")
            .setNumber(mode);
    }

    public boolean hasValidTarget() {
        return LimelightHelpers.getTV(LIMELIGHT);
    }

    public double getTargetTX() {
        return LimelightHelpers.getTX(LIMELIGHT);
    }

    public double getTargetTY() {
        return LimelightHelpers.getTY(LIMELIGHT);
    }

    public double getDistanceToAprilTag() {
        if (!hasValidTarget()) return 0.0;

        double[] targetPose = NetworkTableInstance.getDefault()
            .getTable(LIMELIGHT)
            .getEntry("targetpose_cameraspace")
            .getDoubleArray(new double[6]);

        if (targetPose.length >= 3) {
            double x = targetPose[0], y = targetPose[1], z = targetPose[2];
            if (x != 0.0 || y != 0.0 || z != 0.0) {
                return Math.sqrt(x * x + y * y + z * z);
            }
        }
        return 0.0;
    }

    public void setPipeline(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(LIMELIGHT, pipelineIndex);
    }

    public int getPipeline() {
        return (int) NetworkTableInstance.getDefault()
            .getTable(LIMELIGHT)
            .getEntry("getpipe")
            .getDouble(0.0);
    }

    public void setLEDMode(int mode) {
        NetworkTableInstance.getDefault()
            .getTable(LIMELIGHT)
            .getEntry("ledMode")
            .setNumber(mode);
    }

    public int getLEDMode() {
        return (int) NetworkTableInstance.getDefault()
            .getTable(LIMELIGHT)
            .getEntry("ledMode")
            .getDouble(0.0);
    }
}

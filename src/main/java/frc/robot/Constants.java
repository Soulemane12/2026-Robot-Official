package frc.robot;

/**
 * Constants for the robot.
 * Contains AprilTag alignment PID values, setpoints, and tolerances.
 */
public final class Constants {

    // AprilTag Alignment PID Constants
    // TUNE THESE VALUES FOR YOUR ROBOT!

    // X-axis (forward/backward movement)
    public static final double X_APRILTAG_ALIGNMENT_P = 2.0;  // Start with 2.0, tune as needed
    public static final double X_SETPOINT_APRILTAG_ALIGNMENT = 1.0;  // Target distance in meters from tag
    public static final double X_TOLERANCE_APRILTAG_ALIGNMENT = 0.05;  // 5cm tolerance

    // Y-axis (left/right strafe movement)
    public static final double Y_APRILTAG_ALIGNMENT_P = 2.0;  // Start with 2.0, tune as needed
    public static final double Y_SETPOINT_APRILTAG_ALIGNMENT = 0.0;  // Centered on tag
    public static final double Y_TOLERANCE_APRILTAG_ALIGNMENT = 0.05;  // 5cm tolerance

    // Rotation (turning to face tag)
    public static final double ROT_APRILTAG_ALIGNMENT_P = 0.1;  // Start with 0.1, tune as needed
    public static final double ROT_SETPOINT_APRILTAG_ALIGNMENT = 0.0;  // Face tag head-on (0 degrees)
    public static final double ROT_TOLERANCE_APRILTAG_ALIGNMENT = 2.0;  // 2 degree tolerance

    // Timing constants
    public static final double DONT_SEE_TAG_WAIT_TIME = 1.0;  // Stop if tag lost for 1 second
    public static final double POSE_VALIDATION_TIME = 0.3;  // Must be aligned for 0.3 seconds to finish
}

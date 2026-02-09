package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;

/**
 * Constants for the robot.
 * Contains AprilTag alignment PID values, setpoints, and tolerances.
 */
public final class Constants {
    public static final class CANIds {
        public static final int SHOOTER_MOTOR = 20;
    }

    // AprilTag Alignment PID Constants
    // TUNE THESE VALUES FOR YOUR ROBOT!

    // X-axis (forward/backward movement)
    public static final double X_APRILTAG_ALIGNMENT_P = 2.0;  // Start with 2.0, tune as needed
    public static final double X_SETPOINT_APRILTAG_ALIGNMENT = 1.5;  // Target distance in meters from tag
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

    /**
     * Drive constants for hub alignment and robot control.
     * Contains hub positions, PID configuration, and alignment geometry.
     */
    public static final class DriveConstants {
        // Maximum speeds
        public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        // Shooter offset for alignment geometry (6 inches to the side of robot center)
        public static final Distance shooterSideOffset = Inches.of(6.0);

        // Transform from robot center to shooter position
        public static final Transform2d shooterTransform = new Transform2d(
            Inches.of(0.0),
            shooterSideOffset,
            new Rotation2d()
        );

        // Field hub positions (adjust these to match your field specifications)
        public static final Pose3d redHubPose = new Pose3d(
            Inches.of(468.56),
            Inches.of(158.32),
            Inches.of(72.0),
            new Rotation3d()
        );

        public static final Pose3d blueHubPose = new Pose3d(
            Inches.of(152.56),
            Inches.of(158.32),
            Inches.of(72.0),
            new Rotation3d()
        );

        /**
         * Gets the hub pose for the current alliance.
         * @return The hub pose for red or blue alliance
         */
        public static Pose3d getHubPose() {
            return DriverStation.getAlliance().equals(Optional.of(Alliance.Red))
                ? redHubPose
                : blueHubPose;
        }

        // PID controller for rotation alignment
        public static final PIDController rotationController = getRotationController();

        private static PIDController getRotationController() {
            PIDController controller = new PIDController(2.0, 0.0, 0.0);
            controller.enableContinuousInput(-Math.PI, Math.PI);
            return controller;
        }

        // PID controller for vision alignment (Limelight TX -> rotation)
        public static final PIDController visionRotationController = getVisionRotationController();

        private static PIDController getVisionRotationController() {
            // TX is in degrees, output is angular velocity multiplier (0-1 range)
            // kP = 0.1 means 10 degree error = 1.0 (full speed rotation)
            // This gets multiplied by maxAngularRate later
            PIDController controller = new PIDController(0.1, 0.0, 0.005);
            controller.setTolerance(1.0); // 1 degree tolerance
            return controller;
        }
    }

    /**
     * Limelight vision constants.
     * Contains network table name, mounting position, and camera properties.
     */
    public static final class VisionConstants {
        // Limelight network table name
        public static final String LIMELIGHT_NAME = "limelight";
        // Static IP for Limelight (default 10.TE.AM.11 based on team number)
        public static final String LIMELIGHT_IP = "10.45.71.11";

        // Target AprilTag ID for auto vision tracking
        public static final int TARGET_APRILTAG_ID = 11;

        // Camera mounting position relative to robot center
        // Front center of the robot
        public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
            new Translation3d(
                Inches.of(12.0),  // 12 inches forward from robot center
                Inches.of(0.0),   // Centered left/right
                Inches.of(24.0)   // 24 inches up from ground
            ),
            new Rotation3d(0, Math.toRadians(-20), 0) // Tilted down 20 degrees
        );

        // Camera properties for simulation
        public static final double HORIZONTAL_FOV_DEGREES = 59.6; // Limelight 2/3 horizontal FOV
        public static final double VERTICAL_FOV_DEGREES = 49.7;   // Limelight 2/3 vertical FOV
        public static final int CAMERA_RESOLUTION_WIDTH = 960;
        public static final int CAMERA_RESOLUTION_HEIGHT = 720;
        public static final double MAX_LED_RANGE_METERS = 10.0;  // Maximum effective range
    }
}

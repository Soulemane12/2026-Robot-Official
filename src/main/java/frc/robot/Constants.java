package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

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
        public static final int SHOOTER_MOTOR = 29;
        public static final int INTAKE_PIVOT= 21;
        public static final int INTAKE_ROLLER = 7;
        public static final int TURRET_MOTOR = 26;
        public static final int SHOOTER_ANGLE_MOTOR = 34;
        public static final int ROLLER_TO_SHOOTER = 14;
        public static final int CLIMBER_MOTOR = 12;
        public static final int INDEXER_MOTOR = 30;
    }

    public static final class IndexerConstants {
        public static final double VOLTAGE = -3.0; // tune
    }

    public static final class ClimberConstants {
        public static final double CLIMB_VOLTAGE = 6.0;
        // Position presets (rotations) — jog to find real values, then update these
        public static final double ZERO_POS  = 0.0;
        public static final double CLIMB_POS = -30.36; // TODO: measure on robot
        // PID / MotionMagic — tune on robot
        public static final double kP         = 2.0;
        public static final double kD         = 0.0;
        public static final double kS         = 0.25;
        public static final double kV         = 0.12;
        public static final double CRUISE_RPS = 40.0;
        public static final double ACCEL_RPS2 = 80.0;
        public static final double JERK_RPS3  = 400.0;
    }

    public static final class RollerToShooterConstants {
        public static final double VOLTAGE = 8.0; // tune
    }


    public static final class IntakeConstants {
        public static final double STOW = 0;
        public static final double INTAKE_POSITION = 15;
        public static final double JOG_VOLTAGE = 2.0;
        public static final double ROLLER_VOLTAGE = 5.0;
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

    public static final class TurretConstants {
        public static final double GEAR_RATIO          = 44.0;
        public static final double MIN_DEG             = 0; 
        public static final double MAX_DEG             =  70; 
        public static final double JOG_VOLTAGE         = 2.0;
        public static final double MANUAL_DEADBAND     = 0.08;
        public static final double ANGLE_TOLERANCE_DEG = 1.5;
        public static final double STATOR_LIMIT_A      = 80.0;
        public static final double SUPPLY_LIMIT_A      = 50.0;
        public static final double SUPPLY_THRESHOLD_A  = 70.0;
        public static final double SUPPLY_TIME_S       = 0.2;
        public static final double kP          = 30.0;
        public static final double kD          = 0.2;
        public static final double kS          = 0.4;
        public static final double kV          = 0.12;
        public static final double CRUISE_RPS  = 6.0;
        public static final double ACCEL_RPS2  = 15.0;
        public static final double JERK_RPS3   = 150.0;
    }

    public static final class ShooterAngleConstants {
        public static final double MIN_DEG = 0;
        public static final double MAX_DEG = 55.0;
        public static final double DEG_A   = 0.0;
        public static final double ROT_A   = 0.0;
        public static final double DEG_B   = 55.0;  // TODO: measure on real robot
        public static final double ROT_B   = -5.25; // Hood raises as motor rotations decrease
        public static final double M_ROT_PER_DEG = (ROT_B - ROT_A) / (DEG_B - DEG_A);
        public static final double B_ROT         = ROT_A - M_ROT_PER_DEG * DEG_A;
        public static final double JOG_VOLTAGE         = 6.0;
        public static final double MANUAL_DEADBAND     = 0.08;
        public static final double ANGLE_TOLERANCE_DEG = 1.0;
        public static final double STATOR_LIMIT_A      = 100.0;
        public static final double SUPPLY_LIMIT_A      = 50.0;
        public static final double SUPPLY_THRESHOLD_A  = 70.0;
        public static final double SUPPLY_TIME_S       = 0.2;
        public static final double kP          = 35.0;
        public static final double kD          = 0.3;
        public static final double kS          = 0.5;
        public static final double kV          = 0.12;
        public static final double CRUISE_RPS  = 4.0;
        public static final double ACCEL_RPS2  = 10.0;
        public static final double JERK_RPS3   = 120.0;
    }

    /**
     * Distance-to-shooter lookup table.
     * Maps horizontal distance to target (meters) → shooter voltage and hood angle.
     * TODO: tune all values on the real robot.
     */
    public static final class ShooterTable {
        private static final InterpolatingDoubleTreeMap VOLTAGE_MAP = new InterpolatingDoubleTreeMap();
        private static final InterpolatingDoubleTreeMap ANGLE_MAP   = new InterpolatingDoubleTreeMap();

        static {
            // dist (m) → shooter voltage (V) — matched to measured distances — TODO: tune on robot
            VOLTAGE_MAP.put(1.30,  6.5); // tune
            VOLTAGE_MAP.put(1.80,  7.0); // tune
            VOLTAGE_MAP.put(2.05,  8.5); // tune
            VOLTAGE_MAP.put(2.27,  9.5); // tune — 11.13V was too high
            VOLTAGE_MAP.put(2.44,  9.8); // measured
            VOLTAGE_MAP.put(3.32, 11.5); // tune

            // dist (m) → hood angle (deg) — increases with distance

            ANGLE_MAP.put(1.30, 18.32); // *measured
            ANGLE_MAP.put(1.80, 12.0);  // *measured — shot over at 18°, tuned down
            ANGLE_MAP.put(2.05, 18.32); // *measured
            ANGLE_MAP.put(2.27, 16.0);  // tuned down from 19.62°
            ANGLE_MAP.put(2.44, 17.78); // measured
            ANGLE_MAP.put(3.32, 26.57); // *measured
            
            
        }

        public static double getVoltage(double distM) { return VOLTAGE_MAP.get(distM); }
        public static double getAngleDeg(double distM) { return ANGLE_MAP.get(distM); }
    }

    public static final class VisionConstants {
        public static final String LIMELIGHT_FRONT   = "limelight-front";
        public static final String LIMELIGHT_SIDE    = "limelight-side";
        public static final String LIMELIGHT_TURRET  = "limelight-turret";
        public static final int    APRILTAG_PIPELINE = 0;
        public static final double TX_TO_TURRET_SIGN         = 1.0;
        public static final double TRACKING_ZERO_OFFSET_DEG  = 0.0;
        public static final double TX_DEADBAND_DEG   = 0.7;
        // HUB AprilTag IDs for 2026 REBUILT - all faces of the HUB
        public static final int[]  BLUE_HUB_TAG_IDS = {2, 3, 4, 5, 8, 9, 10, 11};
        public static final int[]  RED_HUB_TAG_IDS  = {18, 19, 20, 21, 24, 25, 26, 27};

        // Use all hub tags until on-field testing confirms which IDs face which direction
        public static final int[]  BLUE_TRACK_TAG_IDS = BLUE_HUB_TAG_IDS;
        public static final int[]  RED_TRACK_TAG_IDS  = RED_HUB_TAG_IDS;
        public static final int[]  BLUE_FERRY_TAG_IDS = BLUE_HUB_TAG_IDS;
        public static final int[]  RED_FERRY_TAG_IDS  = RED_HUB_TAG_IDS;
        // 2026 REBUILT specific vision constants
        public static final double MAX_TAG_DIST_M    = 6.0;  // Increased for larger field
        public static final double BASE_STD_DEV      = 0.5;
        public static final double STD_DEV_PER_METER = 0.4;
    }
}

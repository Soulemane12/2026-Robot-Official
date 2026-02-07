package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;

public final class Constants {

    public static final class CANIds {
        public static final int SHOOTER_MOTOR = 20;
    }

    public static final double X_APRILTAG_ALIGNMENT_P = 2.0;
    public static final double X_SETPOINT_APRILTAG_ALIGNMENT = 1.5;
    public static final double X_TOLERANCE_APRILTAG_ALIGNMENT = 0.05;

    public static final double Y_APRILTAG_ALIGNMENT_P = 2.0;
    public static final double Y_SETPOINT_APRILTAG_ALIGNMENT = 0.0;
    public static final double Y_TOLERANCE_APRILTAG_ALIGNMENT = 0.05;

    public static final double ROT_APRILTAG_ALIGNMENT_P = 0.1;
    public static final double ROT_SETPOINT_APRILTAG_ALIGNMENT = 0.0;
    public static final double ROT_TOLERANCE_APRILTAG_ALIGNMENT = 2.0;

    public static final double DONT_SEE_TAG_WAIT_TIME = 1.0;
    public static final double POSE_VALIDATION_TIME = 0.3;

    public static final class DriveConstants {
        public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        public static final Distance shooterSideOffset = Inches.of(6.0);

        public static final Transform2d shooterTransform = new Transform2d(
            Inches.of(0.0),
            shooterSideOffset,
            new Rotation2d()
        );

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

        public static Pose3d getHubPose() {
            return DriverStation.getAlliance().equals(Optional.of(Alliance.Red))
                ? redHubPose
                : blueHubPose;
        }

        public static final PIDController rotationController = getRotationController();

        private static PIDController getRotationController() {
            PIDController controller = new PIDController(2.0, 0.0, 0.0);
            controller.enableContinuousInput(-Math.PI, Math.PI);
            return controller;
        }

        public static final PIDController visionRotationController = getVisionRotationController();

        private static PIDController getVisionRotationController() {
            PIDController controller = new PIDController(0.1, 0.0, 0.005);
            controller.setTolerance(1.0);
            return controller;
        }
    }
}
